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


#include <rws/RobWorkStudio.hpp>
#include <rwlibs/drawable/RenderFrame.hpp>

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/use_robwork_namespace.hpp>

#include <vector>

#include <QLayout>
#include <QVariant>
#include <QTreeWidgetItem>
#include <QInputDialog>

using namespace robwork;
using namespace rwlibs::drawable;


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
    _workcell(NULL),
    _workcellGLDrawer(NULL),
    _renderFrame(rw::common::Ptr<RenderFrame>(boost::shared_ptr<RenderFrame>((RenderFrame*)NULL)))
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

    QActionGroup* displayTypes = new QActionGroup(this); // owned
    _showWorkCellStructureAction = new QAction(QIcon(":/images/workcell.png"), "WorkCell Structure", displayTypes); // owned
    _showWorkCellStructureAction->setCheckable(true);
    _showWorkCellStructureAction->setChecked(true);
    connect(_showWorkCellStructureAction,
            SIGNAL(triggered()),
            this,
            SLOT(showWorkCellStructure()));

    _showDeviceStructureAction = new QAction(QIcon(":/images/device.png"), "Device Structure", displayTypes); // owned
    _showDeviceStructureAction->setCheckable(true);

    connect(_showDeviceStructureAction,
            SIGNAL(triggered()),
            this,
            SLOT(showDeviceStructure()));

    _showFrameStructureAction = new QAction(QIcon(":/images/frame.png"), "Frame Structure", displayTypes); // owned
    _showFrameStructureAction->setCheckable(true);

    connect(_showFrameStructureAction,
            SIGNAL(triggered()),
            this,
            SLOT(showFrameStructure()));

    toolbar->addAction(collapseAllAction);
    toolbar->addAction(expandAllAction);
    toolbar->addSeparator();
    toolbar->addAction(_showWorkCellStructureAction);
    toolbar->addAction(_showDeviceStructureAction);
    toolbar->addAction(_showFrameStructureAction);

    // Setup TreeWidget
    _treewidget = new QTreeWidget();
    lay->addWidget(_treewidget); // own treewidget
    _treewidget->setColumnCount(1);
    QTreeWidgetItem* header = _treewidget->headerItem();
    _treewidget->setItemHidden(header, true);

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


//    connect(this, SIGNAL(frameSelectedSignalIn(rw::kinematics::Frame*)), this,
//      SLOT(frameSelectedSlot(rw::kinematics::Frame*)));
}

TreeView::~TreeView()
{
   // Q_CLEANUP_RESOURCE(resources);
}

void TreeView::initialize() {
    _renderFrame = rw::common::Ptr<RenderFrame>(boost::shared_ptr<RenderFrame>(new RenderFrame()));
}

void TreeView::frameSelectedHandler(rw::kinematics::Frame* frame, RobWorkStudio* sender) {

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
    const std::vector<Drawable*>& drawables =
        _workcellGLDrawer->getDrawablesForFrame(frame);

    typedef std::vector<Drawable*>::const_iterator DI;
    for (DI p = drawables.begin(); p != drawables.end(); ++p) {
        Drawable* drawable = *p;
        RW_ASSERT(drawable);
        QTreeWidgetItem* item = new QTreeWidgetItem(parent); // owned.
        if ( dynamic_cast<RenderFrame*>(drawable->getRender().get()) ){
            item->setText(0, strVisFrameName);
        } else {
            item->setText(0, "Geometry");
        }
        _drawableMap.insert(make_pair(item, drawable));
        item->setIcon(0, QIcon(":/images/drawable.png"));
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
        const std::vector<Device*>& devices = _workcell->getDevices();
        typedef std::vector<Device*>::const_iterator MI;
        for (MI it = devices.begin(); it != devices.end(); ++it) {
            SerialDevice* sdevice = _convert->toSerialDevice(*it);
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

                    if (_convert->toJoint(frame))
                        frameItem->setIcon(0, QIcon(":/images/joint.png"));
                    else
                        frameItem->setIcon(0, QIcon(":/images/frame.png"));

                    // Setup drawables
                    setupDrawables(frame, frameItem);
                }
                continue;
            }
            TreeDevice* tdevice = _convert->toTreeDevice(*it);
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

                    if (_convert->toJoint(frame))
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
        const std::vector<Device*>& devices = _workcell->getDevices();
        typedef std::vector<Device*>::const_iterator MI;
        for (MI it = devices.begin(); it != devices.end(); ++it) {
            SerialDevice* sdevice = _convert->toSerialDevice(*it);
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
            TreeDevice* tdevice = _convert->toTreeDevice(*it);
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
            ParallelDevice* pdevice = _convert->toParallelDevice(*it);
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
            MobileDevice* mdevice = _convert->toMobileDevice(*it);
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
        _contextMenu->addAction(_selectFrameAction);
        _contextMenu->addAction(_addFrameAction);
        _contextMenu->addSeparator();
    }

    _contextMenu->addAction(_toggleAction);
    _contextMenu->addAction(_showSolidAction);
    _contextMenu->addAction(_showWireAction);
    _contextMenu->addAction(_showOutlineAction);
    _contextMenu->addAction(_showTransparentAction);
    _contextMenu->addAction(_highlightAction);
    _contextMenu->addAction(_poseAction);
    //_contextMenu->addAction(_scaleAction);

    _contextMenu->addSeparator();

    _contextMenu->popup(QWidget::mapToGlobal(pos));
}

void TreeView::toggleFrameView(QTreeWidgetItem* item)
{
	RW_ASSERT(item);

    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        Frame* frame = frameIt->second;
        RW_ASSERT(frame);

        FrameToDrawableMap::iterator frameToDrawableIt =
            _frameToDrawableMap.find(frame);

        if (frameToDrawableIt == _frameToDrawableMap.end()) { // Add new Drawable
            //Drawable* drawable = new DrawableFrame(0.5);
        	Drawable* drawable = new Drawable(_renderFrame);
        	drawable->setScale(0.5);
            RW_ASSERT(_workcellGLDrawer);
            _workcellGLDrawer->addDrawableToFrame(frame, drawable); // own drawable

            QTreeWidgetItem* geoitem = new QTreeWidgetItem(item); // owned.
            geoitem->setText(0, strVisFrameName);
            geoitem->setIcon(0,QIcon(":/images/drawable.png"));

            _drawableMap.insert(make_pair(geoitem, drawable));
            _frameToDrawableMap.insert(
                make_pair(frame, make_pair(drawable, geoitem)));

        } else { // Remove the DrawableFrame

            Drawable* drawable = frameToDrawableIt->second.first;
            RW_ASSERT(drawable);

            QTreeWidgetItem* geoitem = frameToDrawableIt->second.second;
            RW_ASSERT(geoitem);

            // Remove the Drawable from the Frame. This releases ownership, so
            // we delete it also.
            RW_ASSERT(_workcellGLDrawer);
            _workcellGLDrawer->removeDrawableFromFrame(frame, drawable);
            delete drawable;

            // Remove the associated QTreeWidgetItem from tree
            for (int i = item->childCount() - 1; i >= 0; i--) {
                RW_ASSERT(item->child(i));
                if (item->child(i)->text(0) == strVisFrameName) {
                    delete item->takeChild(i);
                    break;
                }
            }

            // Remove it from the QTreeWidgetItem to Drawable map
            typedef DrawableMap::iterator DI;
            DI itemToDrawableIt = _drawableMap.find(geoitem);

            if (itemToDrawableIt != _drawableMap.end())
                _drawableMap.erase(itemToDrawableIt);

            // Remove it from the Frame to Drawable Map
            _frameToDrawableMap.erase(frameToDrawableIt);
        }
    }
}

void TreeView::toggleFrameSlot()
{
    toggleFrameView(_treewidget->currentItem());
    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::scaleSlot()
{
	// create the gui to change the scale of all drawables of a frame

}

void TreeView::constructDrawableList(std::vector<Drawable*>& drawables)
{
    _state = getRobWorkStudio()->getState();
    QList<QTreeWidgetItem*> selected = _treewidget->selectedItems();
    for (int i = 0; i < selected.size(); ++i) {
        QTreeWidgetItem* item = selected.at(i);
        DeviceMap::iterator devIt = _deviceMap.find(item);
        if (devIt != _deviceMap.end()) {
            SerialDevice* sdev = _convert->toSerialDevice((*devIt).second);
            if (sdev != NULL && sdev->frames().size() > 0) {
                Frame* base = sdev->frames().front();
                //assert(_state);
                _workcellGLDrawer->getAllDrawables(_state, base, drawables);
            }
        }

        FrameMap::iterator frameIt = _frameMap.find(item);
        if (frameIt != _frameMap.end()) {
            const std::vector<Drawable*>& frameDrawables =
                _workcellGLDrawer->getDrawablesForFrame(frameIt->second);

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
    std::vector<Drawable*> drawables;
    constructDrawableList(drawables);

    typedef std::vector<Drawable*>::iterator I;
    for (I it = drawables.begin(); it != drawables.end(); ++it)
        (*it)->setDrawType(Render::SOLID);
    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::toggleSlot()
{
    std::vector<Drawable*> drawables;
    constructDrawableList(drawables);

    typedef std::vector<Drawable*>::iterator I;
    for (I it = drawables.begin(); it != drawables.end(); ++it)
        (*it)->setEnabled( !((*it)->isEnabled()) );
    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showWireSlot()
{
    std::vector<Drawable*> drawables;
    constructDrawableList(drawables);

    typedef std::vector<Drawable*>::iterator I;
    for (I it = drawables.begin(); it != drawables.end(); ++it)
        (*it)->setDrawType(Render::WIRE);
    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showOutlineSlot()
{
    std::vector<Drawable*> drawables;
    constructDrawableList(drawables);

    typedef std::vector<Drawable*>::iterator DI;
    for (DI it = drawables.begin(); it != drawables.end(); ++it)
        (*it)->setDrawType(Render::OUTLINE);

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::showTransparentSlot()
{
    bool ok = false;
    const float alpha = (float)QInputDialog::getDouble(
        this, "Select Alpha", "Alpha:", 0.5, 0, 1, 1, &ok);

    if (ok) {
        std::vector<Drawable*> drawables;
        constructDrawableList(drawables);

        typedef std::vector<Drawable*>::iterator DI;
        for (DI it = drawables.begin(); it != drawables.end(); ++it)
            (*it)->setAlpha(alpha);
    }

    getRobWorkStudio()->updateAndRepaint();
}

void TreeView::highlightSlot()
{
    std::vector<Drawable*> drawables;
    constructDrawableList(drawables);

    typedef std::vector<Drawable*>::iterator DI;
    for (DI it = drawables.begin(); it != drawables.end(); ++it)
        (**it).setHighlighted(!(**it).isHighlighted());

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
    }
}

void TreeView::addFrameSlot(){
    // Open a config menu where parent frame is the highlighted frame
    QTreeWidgetItem* item = _treewidget->currentItem();
    FrameMap::iterator frameIt = _frameMap.find(item);
    if (frameIt != _frameMap.end()) {
        Frame* frame = frameIt->second;


        //getRobWorkStudio()->frameSelectedEvent().fire(frame);
    } else {
        //std::cout<<"Frame not found"<<std::endl;
    }
}

void TreeView::open(WorkCell* workcell)
{
    _workcell = workcell;
    _state = getRobWorkStudio()->getState();
    _workcellGLDrawer = getRobWorkStudio()->getWorkCellGLDrawer();
    _treewidget->setHeaderLabels(QStringList("WorkCell"));

    if (_showWorkCellStructureAction->isChecked())
        showWorkCellStructure();
    else if (_showDeviceStructureAction->isChecked())
        showDeviceStructure();
    else if (_showFrameStructureAction->isChecked())
        showFrameStructure();
}

void TreeView::setupFrame(Frame& frame, QTreeWidgetItem* parentItem)
{
    _state = getRobWorkStudio()->getState();
    QTreeWidgetItem* item = new QTreeWidgetItem();
    if (parentItem != NULL)
        parentItem->addChild(item); // own item
    else
        _treewidget->addTopLevelItem(item); // own item
    std::string name = getFrameName(frame);
    item->setText(0, name.c_str());
    registerFrameItem(&frame, item);
    if (_convert->toJoint(&frame))
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
    _workcellGLDrawer = NULL;

    _treewidget->setHeaderLabels(QStringList("WorkCell"));
}

#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(treeview, TreeView)
#endif
