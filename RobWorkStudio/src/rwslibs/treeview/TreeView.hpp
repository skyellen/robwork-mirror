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
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace graphics { class DrawableNode; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class Device; } }

class QAction;
class QTreeWidget;
class QTreeWidgetItem;

namespace rws {

/**
 * @brief The TreeView plugin display the kinematic structure of a workcell in
 * a treeview fashion and enables the user to select and do simple manipulation of frames,
 * devices, drawables and so on.
 */
class TreeView: public RobWorkStudioPlugin {
Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
#endif
public:

    //! constructor
    TreeView();

    //! destructor
    virtual ~TreeView();

    //! @copydoc RobWorkStudioPlugin::open
    virtual void open(rw::models::WorkCell* workcell);

    //! @copydoc RobWorkStudioPlugin::close
    virtual void close();

    /**
     * @brief Update the view - this method can be called from other threads.
     * @param val [in] not currently used.
     */
    void workcellChangedListener(int val);

    /**
     * @brief Select a specific frame.
     * @param frame [in] the frame.
     * @note This is not currently used.
     */
    void frameSelectedListener(rw::kinematics::Frame* frame);

    /**
     * @brief Listen for updates to the state (for instance attaching/detaching of frames).
     * @param state [in] the state.
     */
    void stateChangedListener(const rw::kinematics::State& state);

protected:
	//void frameSelectedHandler(rw::kinematics::Frame* frame, RobWorkStudio* sender);

    //! @copydoc RobWorkStudioPlugin::initialize
	void initialize();
private slots:
    void customContextMenuRequestSlot(const QPoint& pos);
    void toggleFrameSlot();
    void toggleFramesSlot();
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

    void addFromFileSlot();

    // Slots for ToolBar Items
    void collapseAll();
    void expandAll();
    void showWorkCellStructure();
    void showDeviceStructure();
    void showFrameStructure();

    void update();

    void keyPressEvent ( QKeyEvent * event );
private:
    void toggleFrameView(QTreeWidgetItem* item);
    void toggleFramesView(QTreeWidgetItem* item);
    void collapseAll(QTreeWidgetItem* item);
    void expandAll(QTreeWidgetItem* item);

    void clearTreeContent();
    void setupFrame(rw::kinematics::Frame& frame, QTreeWidgetItem* parentItem);
    void constructDrawableList(std::vector<rw::common::Ptr<rw::graphics::DrawableNode> >& drawables);

    void setupDrawables(rw::kinematics::Frame* frame, QTreeWidgetItem* parent);

    void registerFrameItem(rw::kinematics::Frame* frame, QTreeWidgetItem* item);


private:
    // ToolBar Actions
    QAction* _showWorkCellStructureAction;
    QAction* _showDeviceStructureAction;
    QAction* _showFrameStructureAction;

    // Context Menu Actions
    QAction* _toggleFrameAction;
    QAction* _toggleFramesAction;
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

	typedef std::map<QTreeWidgetItem*, rw::common::Ptr<rw::models::Device> > DeviceMap;
    DeviceMap _deviceMap;

    typedef std::map<QTreeWidgetItem*, rw::kinematics::Frame*> FrameMap;
    FrameMap _frameMap;

    typedef std::map<QTreeWidgetItem*, rw::common::Ptr<rw::graphics::DrawableNode> > DrawableMap;
    // maintains the drawables that are not constructed and added from this plugin
    DrawableMap _drawableMap;

    //typedef std::map<rw::graphics::DrawableNode::Ptr, QTreeWidgetItem*> DrawableToItemMap;
    // maintains all drawables that are added by this map
    //DrawableToItemMap _drawableToItemMap;

};
}


#endif //#ifndef TREEVIEW_HPP
