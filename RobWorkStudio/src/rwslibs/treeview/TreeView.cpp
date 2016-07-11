/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "TreeView.hpp"

#include <rw/common/macros.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/MobileDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QLayout>
#include <QVariant>
#include <QTreeWidgetItem>
#include <QInputDialog>
#include <QTreeWidget>
#include <QKeyEvent>
#include <QMenu>
#include <QToolBar>
#include <QFileDialog>

#include <boost/bind.hpp>

#include <vector>

using namespace rw::graphics;
using namespace rws;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::loaders;

using std::make_pair;

const QString strVisFrameName = "Frame Visualization";
const std::string FRAMEID = "TreeView:Frame";

namespace
{
    std::string getFrameName(const Frame& frame)
    {
        std::string name = frame.getName();
        size_t index = name.find_last_of('.');
        if (index == std::string::npos)
            return frame.getName();
        else
            return name.substr(index + 1);
    }

    std::string getFrameNameFirst(const Frame& frame)
    {
        std::string name = frame.getName();
        size_t index = name.find_first_of('.');
        if (index == std::string::npos)
            return frame.getName();
        else
            return name.substr(index + 1);
    }
}

TreeView::TreeView() :
    RobWorkStudioPlugin("TreeView", QIcon(":/treeview.png")),
    _workcell(NULL)

{
	// Construct widget and layout for QDockWidget
	QWidget *widg = new QWidget(this);
	QVBoxLayout *lay = new QVBoxLayout(widg);
	widg->setLayout(lay);
	this->setWidget(widg);

    // Setup ToolBar
    QToolBar* toolbar = new QToolBar();
    lay->addWidget(toolbar); // own toolbar
    toolbar->setIconSize(QSize(12,12));
    lay->setAlignment(toolbar, Qt::AlignTop);

    QAction* collapseAllAction = new QAction(QIcon(":/images/collapse_all.png"), "Collapse All", this); // owned
    connect(collapseAllAction, SIGNAL(triggered()), this, SLOT(collapseAll()));

    QAction* expandAllAction = new QAction(QIcon(":/images/expand_all.png"), "Expand All", this); // owned
    connect(expandAllAction, SIGNAL(triggered()), this, SLOT(expandAll()));

    QAction *forceUpdateAction = new QAction(QIcon(":/images/reload.png"), "Force Update", this); // owned
    connect(forceUpdateAction, SIGNAL(triggered()), this, SLOT(update()));

    QActionGroup* displayTypes = new QActionGroup(this); // owned
    _showWorkCellStructureAction = new QAction(QIcon(":/images/workcell.png"), "WorkCell Structure", displayTypes); // owned
    _showWorkCellStructureAction->setCheckable(true);
    connect(_showWorkCellStructureAction, SIGNAL(triggered()), this, SLOT(showWorkCellStructure()));

    _showDeviceStructureAction = new QAction(QIcon(":/images/device.png"), "Device Structure", displayTypes); // owned
    _showDeviceStructureAction->setCheckable(true);
    connect(_showDeviceStructureAction, SIGNAL(triggered()), this, SLOT(showDeviceStructure()));

    _showFrameStructureAction = new QAction(QIcon(":/images/frame.png"), "Frame Structure", displayTypes); // owned
    _showFrameStructureAction->setCheckable(true);
	_showFrameStructureAction->setChecked(true);
    connect(_showFrameStructureAction, SIGNAL(triggered()), this, SLOT(showFrameStructure()));

    toolbar->addAction(collapseAllAction);
    toolbar->addAction(expandAllAction);
    toolbar->addAction(forceUpdateAction);
    toolbar->addSeparator();
    toolbar->addAction(_showFrameStructureAction);
	toolbar->addAction(_showWorkCellStructureAction);
    toolbar->addAction(_showDeviceStructureAction);

    // Setup TreeWidget
    _treewidget = new QTreeWidget(this);
    lay->addWidget(_treewidget); // own treewidget
    _treewidget->setColumnCount(1);
    QTreeWidgetItem* header = _treewidget->headerItem();
    _treewidget->setItemHidden(header, true);
    connect(_treewidget, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(highlightSlot()));

    // Setup Context Menu
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(
        this,
        SIGNAL(customContextMenuRequested(const QPoint&)),
        this,
        SLOT(customContextMenuRequestSlot(const QPoint&)));

    _contextMenu = new QMenu("Empty", this); // owned

    _toggleFrameAction = new QAction(tr("Show/Remove Frame"), this); // owned
    connect(_toggleFrameAction, SIGNAL(triggered()), this, SLOT(toggleFrameSlot()));
    
    _toggleFramesAction = new QAction(tr("Show/Remove All Frames"), this); // owned
    connect(_toggleFramesAction, SIGNAL(triggered()), this, SLOT(toggleFramesSlot()));

    _selectFrameAction= new QAction(tr("Select Frame"), this); // owned
    connect(_selectFrameAction, SIGNAL(triggered()), this, SLOT(selectFrameSlot()));

    _showSolidAction = new QAction(QIcon(":images/solid.png"), "Solid", this); // owned
    connect(_showSolidAction, SIGNAL(triggered()), this, SLOT(showSolidSlot()));

    _showWireAction = new QAction(QIcon(":/images/wire.png"), "Wire", this); // owned
    connect(_showWireAction, SIGNAL(triggered()), this, SLOT(showWireSlot()));

    _showOutlineAction= new QAction(QIcon(":/images/outline.png"), "Outline", this); // owned
    connect(_showOutlineAction, SIGNAL(triggered()), this, SLOT(showOutlineSlot()));

    _showTransparentAction = new QAction(QIcon(":/images/transparent.png"), "Transparent", this); // owned
    connect(_showTransparentAction, SIGNAL(triggered()), this, SLOT(showTransparentSlot()));

    _highlightAction= new QAction(QIcon(":/images/highlight.png"), "Highlight", this); // owned
    connect(_highlightAction, SIGNAL(triggered()), this, SLOT(highlightSlot()));

    _poseAction = new QAction(QIcon(":/images/pose.png"), "Read pose", this);
    connect(_poseAction, SIGNAL(triggered()), this, SLOT(poseSlot()));

    _toggleAction = new QAction("Toggle enabled", this);
    connect(_toggleAction, SIGNAL(triggered()), this, SLOT(toggleSlot()));


    _addFrameAction = new QAction("Add frame", this);
    connect(_addFrameAction, SIGNAL(triggered()), this, SLOT(addFrameSlot()));

    _scaleAction = new QAction(QIcon(":images/solid.png"), "Scale", this); // owned
    connect(_scaleAction, SIGNAL(triggered()), this, SLOT(scaleSlot()));
}

TreeView::~TreeView()
{
   // Q_CLEANUP_RESOURCE(resources);
}

void TreeView::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&TreeView::stateChangedListener, this, _1), this);

    getRobWorkStudio()->frameSelectedEvent().add(
            boost::bind(&TreeView::frameSelectedListener, this, _1), this);
}

void TreeView::frameSelectedListener(rw::kinematics::Frame* frame) {

}

void TreeView::stateChangedListener(const rw::kinematics::State& state){
    // if the daf state change
//    std::cout << "CHECK FOR DAF CHANGES" << std::endl;

    bool forceUpdate = false;
    BOOST_FOREACH( TreeView::FrameMap::value_type p, _frameMap){
        if( Kinematics::isDAF(p.second) ){
            // test if parent changed
            if(p.second->getParent(state)!=p.second->getParent(_state)){
                forceUpdate = true;
                break;
            }
        }
    }
    if(forceUpdate)
        update();

}

void TreeView::collapseAll(QTreeWidgetItem* item)
{
    if (item != NULL) {
        _treewidget->collapseItem(item);
        for (int i = 0; i < item->childCount(); i++)
            collapseAll(item->child(i));
    }
}

void TreeView::expandAll(QTreeWidgetItem* item)
{
    if (item != NULL) {
        _treewidget->expandItem(item);
        for (int i = 0; i < item->childCount(); i++)
            expandAll(item->child(i));
    }
}

void TreeView::collapseAll()
{
    for (int i = 0; i < _treewidget->topLevelItemCount(); i++) {
        collapseAll(_treewidget->topLevelItem(i));
    }
}

void TreeView::expandAll()
{
    for (int i = 0; i < _treewidget->topLevelItemCount(); i++) {
        expandAll(_treewidget->topLevelItem(i));
    }
}

void TreeView::clearTreeContent()
{
    while (_treewidget->topLevelItemCount() > 0) {
        delete _treewidget->takeTopLevelItem(0);
    }

    _deviceMap.clear();
    _frameMap.clear();
    _drawableMap.clear();
}

void TreeView::setupDrawables(Frame* frame, QTreeWidgetItem* parent)
{
    WorkCellScene::Ptr scene = getRobWorkStudio()->getView()->getWorkCellScene();

    int drawMask = getRobWorkStudio()->getView()->getDrawMask();

    const std::vector<DrawableNode::Ptr>& drawables =
        scene->getDrawables(frame);

    typedef std::vector<DrawableNode::Ptr>::const_iterator DI;
    for (DI p = drawables.begin(); p != drawables.end(); ++p) {
        DrawableNode::Ptr drawable = *p;
        if( !(drawable->getMask() & drawMask) )
            continue;

        RW_ASSERT(drawable);
        QTreeWidgetItem* item = new QTreeWidgetItem(parent); // owned.

        item->setText(0, drawable->getName().c_str());
        _drawableMap[item] = drawable;
        if( drawable->getMask() & DrawableNode::CollisionObject ){
            item->setIcon(0, QIcon(":/images/collision.png"));
        } else {
            item->setIcon(0, QIcon(":/images/drawable.png"));
        }
    }
}

void TreeView::registerFrameItem(Frame* frame, QTreeWidgetItem* item)
{
    _frameMap.insert(make_pair(item, frame));
}

void TreeView::showWorkCellStructure()
{
	clearTreeContent();
    _treewidget->setHeaderLabels(QStringList("WorkCell Structure"));

    if (_workcell != NULL) {
		const std::vector<Device::Ptr>& devices = _workcell->getDevices();
		typedef std::vector<Device::Ptr>::const_iterator MI;
        for (MI it = devices.begin(); it != devices.end(); ++it) {
			SerialDevice::Ptr sdevice = (*it).cast<SerialDevice>();
            if (sdevice) {
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, sdevice));

                deviceItem->setText(0, sdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                const std::vector<Frame*>& frames = sdevice->frames();
                typedef std::vector<Frame*>::const_iterator FI;
                for (FI it = frames.begin(); it != frames.end(); ++it) {
                    Frame* frame = *it;
                    QTreeWidgetItem* frameItem = new QTreeWidgetItem(deviceItem); // owned.
                    frameItem->setText(0, getFrameNameFirst(*frame).c_str());

                    registerFrameItem(frame, frameItem);

                    if (dynamic_cast<Joint*>(frame))
                        frameItem->setIcon(0, QIcon(":/images/joint.png"));
                    else
                        frameItem->setIcon(0, QIcon(":/images/frame.png"));

                    // Setup drawables
                    setupDrawables(frame, frameItem);
                }
                continue;
            }
			TreeDevice::Ptr tdevice = (*it).cast<TreeDevice>();
            if (tdevice) {
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, tdevice));

                deviceItem->setText(0, tdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                const std::vector<Frame*>& frames = tdevice->frames();
                typedef std::vector<Frame*>::const_iterator FI;
                for (FI it = frames.begin(); it != frames.end(); ++it) {
                    Frame* frame = *it;
                    QTreeWidgetItem* frameItem = new QTreeWidgetItem(deviceItem); // owned.
                    frameItem->setText(0, getFrameNameFirst(*frame).c_str());

                    registerFrameItem(frame, frameItem);

                    if (dynamic_cast<Joint*>(frame))
                        frameItem->setIcon(0, QIcon(":/images/joint.png"));
                    else
                        frameItem->setIcon(0, QIcon(":/images/frame.png"));

                    // Setup drawables
                    setupDrawables(frame, frameItem);
                }
                continue;
            }
        }
    }
}

void TreeView::showDeviceStructure()
{
    clearTreeContent();
    _treewidget->setHeaderLabels(QStringList("Device Structure"));

    if (_workcell != NULL) {
		const std::vector<Device::Ptr>& devices = _workcell->getDevices();
		typedef std::vector<Device::Ptr>::const_iterator MI;
        for (MI it = devices.begin(); it != devices.end(); ++it) {
			SerialDevice::Ptr sdevice = (*it).cast<SerialDevice>();
            if (sdevice) {
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, sdevice));
                deviceItem->setText(0, sdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                const std::vector<Frame*>& frames = sdevice->frames();
                if (!frames.empty())
                    setupFrame(*frames.front(), deviceItem);
                continue;
            }
			TreeDevice::Ptr tdevice = (*it).cast<TreeDevice>();
            if(tdevice){
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, tdevice));
                deviceItem->setText(0, tdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                const std::vector<Frame*>& frames = tdevice->frames();
                if (!frames.empty())
                    setupFrame(*frames.front(), deviceItem);
                continue;
            }
			ParallelDevice::Ptr pdevice = (*it).cast<ParallelDevice>();
            if(pdevice){
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, pdevice));
                deviceItem->setText(0, pdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                Frame* frame = pdevice->getBase();
                if (frame!=NULL)
                    setupFrame(*frame, deviceItem);
                continue;
            }
			MobileDevice::Ptr mdevice = (*it).cast<MobileDevice>();
            if(mdevice){
                QTreeWidgetItem* deviceItem = new QTreeWidgetItem();
                _treewidget->addTopLevelItem(deviceItem); // own deviceItem

                _deviceMap.insert(make_pair(deviceItem, mdevice));
                deviceItem->setText(0, mdevice->getName().c_str());
                deviceItem->setIcon(0, QIcon(":/images/device.png"));

                // Setup Frames
                Frame* frame = mdevice->getBase();
                if (frame!=NULL)
                    setupFrame(*frame, deviceItem);
                continue;
            }
        }
    }
    _treewidget->update();
}

void TreeView::showFrameStructure()
{
    clearTreeContent();
    _treewidget->setHeaderLabels(QStringList("Frame Structure"));
    if (_workcell != NULL) {
        setupFrame(*_workcell->getWorldFrame(), NULL);
    }
    _treewidget->update();
}

void TreeView::addFromFileSlot(){
    QString selectedFilter;

    std::string previousOpenDirectory = getRobWorkStudio()->getSettings().get<std::string>("PreviousOpenDirectory","");
    const QString dir(previousOpenDirectory.c_str());

    QString filename = QFileDialog::getOpenFileName(
        this,
        "Open geometry", // Title
        dir, // Directory
        "All supported ( *.stl *.stla *.stlb *.pcd )"
        "\n All ( *.* )",
        &selectedFilter);

    std::string str = filename.toStdString();

    QTreeWidgetItem* item = _treewidget->currentItem();
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        Frame* frame = frameIt->second;
        //getRobWorkStudio()->frameSelectedEvent().fire(frame);

        Geometry::Ptr gdata = GeometryFactory::load( str );
        std::cout << "Geom loaded" << std::endl;
        // create and add a render to the frame
        getRobWorkStudio()->getWorkCellScene()->addGeometry("geo", gdata, frame);

    } else {
        std::cout << "could not find frame..... " << std::endl;
    }


}

void TreeView::customContextMenuRequestSlot(const QPoint& pos)
{
    _contextMenu->clear();

    QTreeWidgetItem* item = _treewidget->currentItem();
    DeviceMap::iterator devIt = _deviceMap.find(item);
    if (devIt != _deviceMap.end()) {
    }

    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        _contextMenu->addAction(_toggleFrameAction);
        _contextMenu->addAction(_toggleFramesAction);
        _contextMenu->addAction(_selectFrameAction);
        //_contextMenu->addAction(_addFrameAction);

        QMenu *addgeomMenu = _contextMenu->addMenu( "add " );

        addgeomMenu->addAction(_addFrameAction);
        connect(addgeomMenu->addAction("geometry file"), SIGNAL(triggered()),this, SLOT(addFromFileSlot()));
        connect(addgeomMenu->addAction("model file"), SIGNAL(triggered()),this, SLOT(addFromFileSlot()));
        connect(addgeomMenu->addAction("primitive"), SIGNAL(triggered()),this, SLOT(addFromFileSlot()));

        //addgeomMenu->addAction("from file")->connect( );

        //_showSolidAction = new QAction(QIcon(":images/solid.png"), "Solid", this); // owned
        //connect(_showSolidAction, SIGNAL(triggered()), this, SLOT(showSolidSlot()));

        _contextMenu->addSeparator();
    }

    _contextMenu->addAction(_toggleAction);
    _contextMenu->addAction(_showSolidAction);
    _contextMenu->addAction(_showWireAction);
    _contextMenu->addAction(_showOutlineAction);
    _contextMenu->addAction(_showTransparentAction);
    _contextMenu->addAction(_highlightAction);
    _contextMenu->addAction(_poseAction);
    _contextMenu->addAction(_scaleAction);

    _contextMenu->addSeparator();

    _contextMenu->popup(QWidget::mapToGlobal(pos));
}

void TreeView::toggleFrameView(QTreeWidgetItem* item)
{
	RW_ASSERT(item);
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        try {
            Frame* frame = frameIt->second;
            RW_ASSERT(frame);
            WorkCellScene::Ptr scene = getRobWorkStudio()->getWorkCellScene();
            RW_ASSERT(scene);
            if ( !scene->isFrameAxisVisible(frame) ) {
                // Add new Drawable
                scene->setFrameAxisVisible(true, frame);

            } else { // Remove the DrawableFrame
                scene->setFrameAxisVisible(false, frame);
            }
        } catch(...){

        }

    }
}



void TreeView::toggleFramesView(QTreeWidgetItem* item)
{
	RW_ASSERT(item);

    toggleFrameView(item);
    
    for (int i = 0; i < item->childCount(); i++) {
            toggleFramesView(item->child(i));
	}
}



void TreeView::toggleFrameSlot()
{
    toggleFrameView(_treewidget->currentItem());
    getRobWorkStudio()->updateAndRepaint();
}


void TreeView::toggleFramesSlot()
{
    toggleFramesView(_treewidget->currentItem());
    getRobWorkStudio()->updateAndRepaint();
}


void TreeView::constructDrawableList(std::vector<DrawableNode::Ptr>& drawables)
{
    WorkCellScene::Ptr scene = getRobWorkStudio()->getView()->getWorkCellScene();
    _state = getRobWorkStudio()->getState();
    QList<QTreeWidgetItem*> selected = _treewidget->selectedItems();
    for (int i = 0; i < selected.size(); ++i) {
        QTreeWidgetItem* item = selected.at(i);
        DeviceMap::iterator devIt = _deviceMap.find(item);
        if (devIt != _deviceMap.end()) {
			SerialDevice::Ptr sdev = (*devIt).second.cast<SerialDevice>();
            if (sdev != NULL && sdev->frames().size() > 0) {
                Frame* base = sdev->frames().front();
                //assert(_state);
                std::vector<DrawableNode::Ptr> newdrawables = scene->getDrawablesRec(base, _state);
                BOOST_FOREACH(DrawableNode::Ptr d,newdrawables){ drawables.push_back(d); }
            }
        }

        FrameMap::iterator frameIt = _frameMap.find(item);
        if (frameIt != _frameMap.end()) {
            const std::vector<DrawableNode::Ptr>& frameDrawables = scene->getDrawables(frameIt->second);

            drawables.insert(
                drawables.end(), frameDrawables.begin(), frameDrawables.end());
        }

        DrawableMap::iterator drawableIt = _drawableMap.find(item);
        if (drawableIt != _drawableMap.end())
            drawables.push_back(drawableIt->second);
    }
}

void TreeView::showSolidSlot()
{
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);

    BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
        node->setDrawType(DrawableNode::SOLID);

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::toggleSlot()
{
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);

    BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
        node->setVisible( !(node->isVisible()) );

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showWireSlot()
{
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);

    BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
        node->setDrawType(DrawableNode::WIRE);

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showOutlineSlot()
{
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);

    BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
        node->setDrawType(DrawableNode::OUTLINE);

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showTransparentSlot()
{
    bool ok = false;
    const float alpha = (float)QInputDialog::getDouble(
        this, "Select Alpha", "Alpha:", 0.5, 0, 1, 1, &ok);

    if (ok) {
        std::vector<DrawableNode::Ptr> drawables;
        constructDrawableList(drawables);

        BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
            node->setTransparency(alpha);
    }

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::scaleSlot()
{
    // create the gui to change the scale of all drawables of a frame
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);
    if(drawables.size()==0)
        return;

    bool ok = false;
    const float scale = (float)QInputDialog::getDouble(
        this, "Select Scale", "Scale:", drawables[0]->getScale(), 0, 1000, 1, &ok);

    if (ok) {
        BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
            node->setScale(scale);
    }

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::highlightSlot()
{
    std::vector<DrawableNode::Ptr> drawables;
    constructDrawableList(drawables);

    BOOST_FOREACH(DrawableNode::Ptr& node, drawables)
        node->setHighlighted(!node->isHighlighted());

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::selectFrameSlot()
{
    QTreeWidgetItem* item = _treewidget->currentItem();
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        Frame* frame = frameIt->second;
        getRobWorkStudio()->frameSelectedEvent().fire(frame);
    } else {
        //std::cout<<"Frame not found"<<std::endl;
    }
}

void TreeView::poseSlot() {
    _state = getRobWorkStudio()->getState();
    //std::cout << "Pose of frame ";
    QTreeWidgetItem* item = _treewidget->currentItem();
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        Frame* frame = frameIt->second;
        log().info() << frame->getName() << ": " << std::endl
					<< rw::kinematics::Kinematics::worldTframe(frame, _state) << std::endl;
        return;
    }
    DrawableMap::iterator drawableIt = _drawableMap.find(item);
    if (drawableIt != _drawableMap.end()) {
        DrawableNode::Ptr drawable = drawableIt->second;
        Frame *frame = getRobWorkStudio()->getWorkCellScene()->getFrame(drawable);
        Transform3D<> t3d;
        if(frame==NULL){
            t3d = drawable->getTransform();
        } else {
            t3d = rw::kinematics::Kinematics::worldTframe(frame, _state) * drawable->getTransform();
        }

        log().info() << drawable->getName() << ": " << std::endl
                    << t3d << std::endl;
        return;
    }

}

void TreeView::addFrameSlot(){
    // Open a config menu where parent frame is the highlighted frame
    QTreeWidgetItem* item = _treewidget->currentItem();
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        //Frame* frame = frameIt->second;
        //getRobWorkStudio()->frameSelectedEvent().fire(frame);
    } else {
        //std::cout<<"Frame not found"<<std::endl;
    }
}

void TreeView::open(WorkCell* workcell)
{
    _workcell = workcell;
    _state = getRobWorkStudio()->getState();
    _treewidget->setHeaderLabels(QStringList("WorkCell"));

	//_frameToDrawableMap.clear();
    if (_showWorkCellStructureAction->isChecked()){
        showWorkCellStructure();
    } else if (_showDeviceStructureAction->isChecked()){
        showDeviceStructure();
    } else if (_showFrameStructureAction->isChecked()){
        showFrameStructure();
    }

    // connect the workcell changed handler
    _workcell->workCellChangedEvent().add(boost::bind(&TreeView::workcellChangedListener, this, _1), this);

}

void TreeView::workcellChangedListener(int){
    // we need to call the update slot, but since this is possibly from a non qt thread, we need to separate
    // it through Qt::queue
//     std::cout << "TreeView: WORKCELL CHANGED" << std::endl;

    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);

}

void TreeView::setupFrame(Frame& frame, QTreeWidgetItem* parentItem)
{
    //State state = getRobWorkStudio()->getState();
    QTreeWidgetItem* item = new QTreeWidgetItem();
    if (parentItem != NULL)
        parentItem->addChild(item); // own item
    else
        _treewidget->addTopLevelItem(item); // own item
    std::string name = getFrameName(frame);
    item->setText(0, name.c_str());

    registerFrameItem(&frame, item);
    if ( dynamic_cast<Joint*>(&frame) )
        item->setIcon(0,QIcon(":/images/joint.png"));
    else
        item->setIcon(0,QIcon(":/images/frame.png"));
    Frame::iterator_pair children = frame.getChildren(_state);
    for (Frame::iterator it = children.first; it != children.second; ++it) {
        setupFrame(*it, item);
    }
    setupDrawables(&frame, item);
}

void TreeView::close()
{
    clearTreeContent();

    _workcell = NULL;
    _treewidget->setHeaderLabels(QStringList("WorkCell"));
}

void TreeView::update(){
    _state = getRobWorkStudio()->getState();
    clearTreeContent();
    //_frameToDrawableMap.clear();
    if (_showWorkCellStructureAction->isChecked()){
        showWorkCellStructure();
    } else if (_showDeviceStructureAction->isChecked()){
        showDeviceStructure();
    } else if (_showFrameStructureAction->isChecked()){
        showFrameStructure();
    }
}

void TreeView::keyPressEvent ( QKeyEvent * event ){
    std::cout << "keyPressedEvent" << std::endl;
    if(event->key()==Qt::Key_Space){
        toggleSlot();
    //} else if(event->key()==Qt::Key_Space){

    } else {
        QDockWidget::keyPressEvent(event);
    }
}


#ifndef RWS_USE_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN2(treeview, TreeView)
#endif
#endif
