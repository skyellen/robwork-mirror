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

#ifndef RW_GRAPHICS_WORKCELLSCENE_HPP_
#define RW_GRAPHICS_WORKCELLSCENE_HPP_

#include "SceneGraph.hpp"

#include <map>

#include <rw/kinematics/FKTable.hpp>

namespace rw { namespace geometry { class PointCloud;}}
namespace rw { namespace kinematics { class Frame; class State; }}
namespace rw { namespace models { class DeformableObject;}}
namespace rw { namespace models { class WorkCell;}}
namespace rw { namespace sensor { class Image;}}

namespace rw {
namespace graphics {

    /**
     * @brief  class for wrapping the SceneGraph interface such that it extends the scene-graph
     * functionality to work on frames and workcells.
     *
     * The scene graph is composed of nodes which are related to Frames. Each frame
     * can have several Drawables attached which can be considered as leafs. Beside keeping track
     * of the mapping from frames to scenenodes, this class also adds settings such as highlighted,
     * adding of frameaxis(per Frame), visibility and so on.
     */
    class WorkCellScene {
    public:
        //! @brief smart pointer to this class
        typedef rw::common::Ptr<WorkCellScene> Ptr;

        /**
         * @brief constructor - creates a workcell with a current SceneGraph
         * @param scene [in] scene graph to use.
         */
        WorkCellScene(SceneGraph::Ptr scene);

        //! @brief destructor
        virtual ~WorkCellScene();

        /**
         * @brief renders the complete scene using the settings in \b info.
         *
         * This basically forwards the call to SceneGraph::draw
         *
         * @param info [in] render settings
         */
        void draw(SceneGraph::RenderInfo& info);

        /**
         * @brief set the workcell to render
         * @param wc [in] the workcell that is to be rendered
         */
        void setWorkCell(rw::common::Ptr<rw::models::WorkCell> wc);


        //! @brief get the workcell that is currently being rendered.
        rw::common::Ptr<rw::models::WorkCell> getWorkCell();

        /**
         * @brief state changes are updated by calling this method. That includes
         * transformations between frames, joints and stuff.
         * @param state [in] the new state that is to be rendered
         */
        void setState(const rw::kinematics::State& state);

        /**
         * @brief gets the node of the scene graph that maps to the world frame
         * of the workcell.
         * @return scene node
         */
        rw::graphics::GroupNode::Ptr getWorldNode();

        /**
         * @brief updates the state of the scenegraph to that of \b state
         * @param state
         */
        void updateSceneGraph(rw::kinematics::State& state);

        /**
         * @brief any cached drawables or scene nodes are deleted
         */
        void clearCache();

        // the frame interface
        /**
         * @brief sets the visibility of a frame and its drawables.
         * @param f [in] the frame.
         * @param visible [in] true if frame should be visible, false otherwise
         */
        void setVisible(bool visible, const rw::kinematics::Frame* f);

        /**
         * @brief test if a frame is visible or not
         * @param f [in] the frame
         * @return true if frame is visible, false if not
         */
        bool isVisible(const rw::kinematics::Frame* f) const;

        /**
         * @brief sets a frame to be highlighted or not.
         * @param f [in] the frame.
         * @param highlighted [in] true if frame should be highlighted, false otherwise
         */
        void setHighlighted(bool highlighted, const rw::kinematics::Frame* f);

        /**
         * @brief test if a frame is highlighted or not
         * @param f [in] the frame
         * @return true if frame is highlighted, false if not
         */
        bool isHighlighted(const rw::kinematics::Frame* f) const;

        /**
         * @brief enables the drawing of the frame-axis of a frame.
         * @param visible [in] true if frame axis should be drawn, false otherwise
         * @param f [in] the frame
         */
        void setFrameAxisVisible(bool visible, rw::kinematics::Frame* f);

        /**
         * @brief test if frame-axis is visible
         * @param f [in] the frame
         * @return true if frame axis of frame is set to be drawn, false otherwise
         */
        bool isFrameAxisVisible(const rw::kinematics::Frame* f) const;

        /**
         * @brief set how drawables of a specific frame should be rendered
         * @param type [in] the drawtype
         * @param f [in] the Frame
         */
        void setDrawType(DrawableNode::DrawType type, const rw::kinematics::Frame* f);

        /**
         * @brief get how drawables of a specific frame is to be rendered
         * @param f [in] the Frame
         * @return the drawtype
         */
        DrawableNode::DrawType getDrawType(const rw::kinematics::Frame* f) const;

        /**
         * @brief set the draw mask of the drawables of a specific frame
         * @param mask [in] draw mask
         * @param f [in] the frame
         */
        void setDrawMask(unsigned int mask, const rw::kinematics::Frame* f);

        /**
         * @brief get the draw mask of the drawables of a specific frame
         * @param f [in] the frame
         * @return the drawmask
         */
        unsigned int getDrawMask(const rw::kinematics::Frame* f) const;

        /**
         * @brief set drawables of a frame to be translusent
         * @param alpha [in] range [0-1] where 1 is fully opaque and 0 is folly transparent
         * @param f [in] frame
         */
        void setTransparency(double alpha, const rw::kinematics::Frame* f);

        //******************************** interface for adding drawables  ***/
        /**
         * @brief create and add a drawable geometry node of line geometry to the scene
         * @param name [in] name of drawable node
         * @param lines [in] the line geometry
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableGeometryNode::Ptr addLines( const std::string& name, const std::vector<rw::geometry::Line>& lines, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable geometry node of any type of geometry to the scene
         * @param name [in] name of drawable node
         * @param geom [in] the geometry
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableGeometryNode::Ptr addGeometry(const std::string& name, rw::common::Ptr<rw::geometry::Geometry> geom, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node of a frame axis to the scene
         * @param name [in] name of drawable node
         * @param size [in] the length of the axis arrows in meters
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableNode::Ptr addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a model3d to the scene
         * @param name [in] name of drawable node
         * @param model [in] the model3d
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         */
        DrawableNode::Ptr addModel3D(const std::string& name, rw::common::Ptr<class Model3D> model, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node of an image to the scene
         * @param name [in] name of drawable node
         * @param img [in] the image
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node geometry
         * @note the size of the image in the scene will be pixel/cm. To change this please use scale.
         */
        DrawableNode::Ptr addImage(const std::string& name, const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a scan to the scene
         * @param name [in] name of drawable node
         * @param scan [in] the scan
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableNode::Ptr addScan(const std::string& name, const rw::geometry::PointCloud& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);

        /**
         * @brief create and add a drawable node of a render, to the scene
         * @param name [in] name of drawable node
         * @param render [in] the render
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableNode::Ptr addRender(const std::string& name, rw::common::Ptr<class Render> render, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

        /**
         * @brief create and add a drawable node from a filename to the scene
         * @param filename [in] name of drawable node
         * @param frame [in] the frame where the drawable is to be placed
         * @param dmask [in] the drawable mask
         * @return the drawable node
         */
        DrawableNode::Ptr addDrawable(const std::string& filename, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);


        /**
         * @brief add a drawable node to the scene
         * @param drawable [in] the drawable
         * @param frame [in] the frame where the drawable is to be placed
         */
        void addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame* frame);

        /**
         * @brief get all drawables of the WorkCellScene
         * @return a list of drawables
         */
        std::vector<DrawableNode::Ptr> getDrawables();

        /**
         * @brief get all drawables of a specific frame in the WorkCellScene
         * @param f [in] the frame
         * @return a list of drawables
         */
        std::vector<DrawableNode::Ptr> getDrawables(const rw::kinematics::Frame* f) const;

        /**
         * @brief get all drawables of a frame and all frames that kinematicly is connected to
         * the sub-tree of \b f.
         * @param f [in] the frame
         * @param state [in] the current state
         * @return a list of drawables
         */
        std::vector<DrawableNode::Ptr> getDrawablesRec(rw::kinematics::Frame* f, rw::kinematics::State& state);

        /**
         * @brief find a drawable by name \b name.
         * @param name [in] the name of the drawable
         * @return a drawable with name \b name or NULL if no such drawable exist in the scene
         */
        DrawableNode::Ptr findDrawable(const std::string& name);

        /**
         * @brief find a drawable by name \b name that is attached to frame \b frame
         * @param name [in] the name of the drawable
         * @param frame [in] search only the subtree beginning from this frame.
         * @return a drawable with name \b name or NULL if no such drawable exist in the scene
         */
        DrawableNode::Ptr findDrawable(const std::string& name, const rw::kinematics::Frame* frame);

        /**
         * @brief find all drawables by name \b name.
         * @param name [in] the name of the drawable
         * @return all drawables with name \b name
         */
        std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

        /**
         * @brief remove all drawables on a frame
         * @param f [in] the frame
         * @return true if successfull
         */
        bool removeDrawables(const rw::kinematics::Frame* f);

        /**
         * @brief remove all drawables with name \n name
         * @param name [in] the name
         * @return true if successfull
         */
        bool removeDrawables(const std::string& name);

        /**
         * @brief remove a drawable from the scene
         * @param drawable [in] the drawable
         * @return true if successfull
         */
        bool removeDrawable(DrawableNode::Ptr drawable);

        /**
         * @brief remove a drawable from a specific frame
         * @param drawable [in] the drawable
         * @param f [in] the frame
         * @return true if successfull
         */
        bool removeDrawable(DrawableNode::Ptr drawable, const rw::kinematics::Frame* f);

        /**
         * @brief remove first drawable by name \b name
         * @param name [in] the name
         * @return true if successfull
         */
        bool removeDrawable(const std::string& name);

        /**
         * @brief remove drawable by name \n and which is attached to frame \b f
         * @param name [in] name of drawable
         * @param f [in] the frame
         * @return true if successfull
         */
        bool removeDrawable(const std::string& name, const rw::kinematics::Frame* f);

        /**
         * @brief get the frame that a specific drawable \b d is associated to
         * @param d [in] the drawable
         * @return the first frame that the drawable is associated to, or NULL if there are no associations
         */
        rw::kinematics::Frame* getFrame(DrawableNode::Ptr d) const;

        /**
         * @brief Get the GroupNode corresponding to the given \b frame.
         * @param frame [in] the frame.
         * @return group node.
         */
        rw::graphics::GroupNode::Ptr getNode(const rw::kinematics::Frame* frame) const;

    private:
        /**
         * @brief listens for changes in the rw::models::WorkCell
         * @param int [in] changed type
         */
        void workCellChangedListener(int);

    private:
        SceneGraph::Ptr _scene;
        rw::common::Ptr<rw::models::WorkCell> _wc;
        rw::kinematics::FKTable _fk;

        typedef std::map<const rw::kinematics::Frame*, GroupNode::Ptr> FrameNodeMap;
        typedef std::map<GroupNode::Ptr, rw::kinematics::Frame*> NodeFrameMap;

        //! the mapping from frame to group nodes 1:1
        FrameNodeMap _frameNodeMap;
        //! the mapping from group nodes to frames 1:1
        NodeFrameMap _nodeFrameMap;

        //! @brief Struct for keeping track of the visual state of each frame between reloads of workcell
        struct FrameVisualState;

        //! mapping from frame to visualization state, 1:1
        std::map<const rw::kinematics::Frame*, FrameVisualState> _frameStateMap;
        //! mapping from frame to all its drawables, 1:many
        std::map<const rw::kinematics::Frame*, std::vector<DrawableNode::Ptr> > _frameDrawableMap;

        std::map<rw::common::Ptr<rw::models::DeformableObject>, std::vector<rw::common::Ptr<class Model3D> >  > _deformableObjectsMap;

        //! the drawable used to draw the frame axis
        DrawableNode::Ptr _frameAxis;
        //! world node
        GroupNode::Ptr _worldNode;
    };
}
}

#endif /* RWSCENEGRAPH_HPP_ */
