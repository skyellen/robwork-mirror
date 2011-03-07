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


#ifndef TREEVIEW_HPP
#define TREEVIEW_HPP

#include <map>

#include <QObject>
#include <QTreeWidget>
#include <QActionGroup>
//#include <QModelIndex>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <QtGui>


namespace rws {

/**
 * @brief The TreeView plugin display the kinematic structure of a workcell in
 * a treeview fashion and enables the user to select and do simple manipulation of frames,
 * devices, drawables and so on.
 */
class TreeView: public RobWorkStudioPlugin {
Q_OBJECT
#ifndef RW_STATIC_LINK_PLUGINS
Q_INTERFACES( rws::RobWorkStudioPlugin )
#endif
public:

    //! constructor
    TreeView();

    //! destructor
    virtual ~TreeView();

    //! @copydoc RobWorkStudioPlugin::open
    virtual void open(rw::models::WorkCell* workcell);

    //! @copydoc RobWorkStudioPlugin::open
    virtual void close();


    void workcellChangedListener(int);

protected:
	void frameSelectedHandler(rw::kinematics::Frame* frame, RobWorkStudio* sender);

	void initialize();
private slots:
    void customContextMenuRequestSlot(const QPoint& pos);
    void toggleFrameSlot();
    void selectFrameSlot();
    void showSolidSlot();
    void showWireSlot();
    void showOutlineSlot();
    void showTransparentSlot();
    void highlightSlot();
    void poseSlot();
    void toggleSlot();
    void addFrameSlot();
    void scaleSlot();

    // Slots for ToolBar Items
    void collapseAll();
    void expandAll();
    void showWorkCellStructure();
    void showDeviceStructure();
    void showFrameStructure();

    void update();

private:
    void toggleFrameView(QTreeWidgetItem* item);
    void collapseAll(QTreeWidgetItem* item);
    void expandAll(QTreeWidgetItem* item);

    void clearTreeContent();
    void setupFrame(rw::kinematics::Frame& frame, QTreeWidgetItem* parentItem);
    void constructDrawableList(std::vector<rw::graphics::DrawableNode::Ptr>& drawables);

    void setupDrawables(rw::kinematics::Frame* frame, QTreeWidgetItem* parent);

    void registerFrameItem(rw::kinematics::Frame* frame, QTreeWidgetItem* item);

private:
    // ToolBar Actions
    QAction* _showWorkCellStructureAction;
    QAction* _showDeviceStructureAction;
    QAction* _showFrameStructureAction;

    // Context Menu Actions
    QAction* _toggleFrameAction;
    QAction* _selectFrameAction;
    QAction* _showSolidAction;
    QAction* _showWireAction;
    QAction* _showOutlineAction;
    QAction* _showTransparentAction;
    QAction* _highlightAction;
    QAction* _poseAction;
    QAction* _toggleAction;
    QAction* _addFrameAction;
    QAction* _scaleAction;

    QTreeWidget* _treewidget;
    QMenu* _contextMenu;

    rw::models::WorkCell* _workcell;
    rw::kinematics::State _state;

	typedef std::map<QTreeWidgetItem*, rw::models::Device::Ptr> DeviceMap;
    DeviceMap _deviceMap;

    typedef std::map<QTreeWidgetItem*, rw::kinematics::Frame*> FrameMap;
    FrameMap _frameMap;

    typedef std::map<QTreeWidgetItem*, rw::graphics::DrawableNode::Ptr> DrawableMap;
    // maintains the drawables that are not constructed and added from this plugin
    DrawableMap _drawableMap;

    typedef std::map<rw::graphics::DrawableNode::Ptr, QTreeWidgetItem*> DrawableToItemMap;
    // maintains all drawables that are added by this map
    DrawableToItemMap _drawableToItemMap;

};
}


#endif //#ifndef TREEVIEW_HPP
