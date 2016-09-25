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

#ifndef RW_GRAPHICS_SCENEGRAPH_HPP_
#define RW_GRAPHICS_SCENEGRAPH_HPP_

#include <rw/graphics/DrawableNode.hpp>
#include <rw/graphics/DrawableGeometryNode.hpp>

#include "SceneCamera.hpp"
#include "GroupNode.hpp"

#include <boost/function.hpp>

namespace rw {
namespace graphics {

    /**
     * @brief interface for a minimalistic scenegraph that does not depend on the frame structure. See
     * WorkCellScene for an implementation that wraps this interface to enable a more RobWork specific
     * use.
     *
     * \b Cameras - SceneCameras define how the world is rendered from a specific point of view,
     * render setting, projection matrix, viewport and so on.
     *
     * Cameras are ordered in CameraGroups. Each camera group can contain multiple cameras that
     * are ordered according to how they should render the scene. A typical setup of one group would be:
     * first camera - clear color and depth buffers, render background ibn orthographic projection
     * second camera - render all 3D scene elements in perspective projection
     * third camera - render foreground such as logo in orthographic projection
     *
     * As can be seen the ordering is of high importance since the first camera clears the frame buffers.
     */
    class SceneGraph {
    public:
        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<SceneGraph> Ptr;

        //! @brief Drawing types.
        typedef DrawableNode::DrawType DrawType;
        /*
        enum DrawType {
            SOLID, //! Render in solid
            WIRE, //! Render in wireframe
            OUTLINE //! Render both solid and wireframe
        };
        */

        /**
         * @brief all general render information is located in this struct
         */
        struct RenderInfo: public DrawableNode::RenderInfo {
        	//! @brief Constructor.
            RenderInfo()
                    //:cameraGroup(0)
            {}
            //int cameraGroup;
            //! @brief Camera groups.
            CameraGroup::Ptr cams;
            //DrawType dtype;
        };

        //! @brief Destructor.
        virtual ~SceneGraph(){}

        /**
         * @brief draws the scene, using the specified render information
         */
        virtual void draw(RenderInfo& info) = 0;

        /**
         * @brief picks the drawable in the scene that intersects with the ray (x,y,-1) in
         * camera coordinates.
         *
         * @param info [in] rendering information.
         * @param x [in] first camera coordinate.
         * @param y [in] second camera coordinate.
         * @return the picked drawable.
         */
        virtual DrawableNode::Ptr pickDrawable(RenderInfo& info, int x, int y) = 0;

        /**
         * @brief this method unprojects a 2D screen coordinate to 3D coordinates from the last draw'n scene.
         * Which is the closest 3d point from the intersection of the ray (x,y,-1) and the objects drawn in the scene.
         *
         * @note this method relies on a previously drawn scene, eg. call to draw(). Any thing drawn in the scene
         * can be "picked" by this method.
         *
         * (0,0) is located in the upper left corner, with x-axis increasing to the right and y-axis
         * increasing to the bottom. The negative z-axis points into the scene-
         * @param camera [in] the scene camera.
         * @param x [in] x coordinate [0;viewport.width]
         * @param y [in] y coordinate [0;viewport.height]
         * @return the 3D point,
         */
        virtual rw::math::Vector3D<> unproject(SceneCamera::Ptr camera, int x, int y) = 0;

        /**
         * @brief should be called after the structure of the scene
         * has been changed
         */
        virtual void update() = 0;

        //! @brief Clear the scene graph.
        virtual void clear() = 0;

        // interface for adding drawables
        /**
         * @brief Create a drawable node for a frame axis coordinate system.
         * @param name [in] name of the drawable.
         * @param size [in] size of the axis.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable geometry node.
         */
        virtual DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask=DrawableNode::Physical)= 0;

        /**
         * @brief Create a drawable node for a geometry.
         * @param name [in] name of the drawable.
         * @param geom [in] the geometry to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable geometry node.
         */
        virtual DrawableGeometryNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class rw::geometry::Geometry> geom, int dmask=DrawableNode::Physical) = 0;

        /**
         * @brief Create a drawable node for lines.
         * @param name [in] name of the drawable.
         * @param lines [in] the lines to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable geometry node.
         */
        virtual DrawableGeometryNode::Ptr makeDrawable(const std::string& name, const std::vector<class rw::geometry::Line >& lines, int dmask=DrawableNode::Physical)= 0;

        /**
         * @brief Create a drawable node for an image.
         * @param name [in] name of the drawable.
         * @param img [in] the image to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable node.
         */
        virtual DrawableNode::Ptr makeDrawable(const std::string& name, const class rw::sensor::Image& img, int dmask=DrawableNode::Virtual)= 0;

        /**
         * @brief Create a drawable node for a point cloud.
         * @param name [in] name of the drawable.
         * @param scan [in] the point cloud to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable node.
         */
        virtual DrawableNode::Ptr makeDrawable(const std::string& name, const rw::geometry::PointCloud& scan, int dmask=DrawableNode::Virtual)= 0;

        /**
         * @brief Create a drawable node for a 3d model.
         * @param name [in] name of the drawable.
         * @param model [in] the 3d model to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable node.
         */
        virtual DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class Model3D> model, int dmask=DrawableNode::Physical) = 0;

        /**
         * @brief Create a drawable node for a render.
         * @param name [in] name of the drawable.
         * @param render [in] the render to draw.
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable node.
         */
        virtual DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class Render> render, int dmask=DrawableNode::Physical)= 0;

        //virtual DrawableNode::Ptr makeDrawable(const rw::models::DrawableModelInfo& info) = 0;
        //virtual DrawableNode::Ptr makeDrawable(const rw::models::CollisionModelInfo& info) = 0;

        /**
         * @brief Create a drawable node from a file.
         * @param filename [in] a filename (alternatively a string for a geometric primitive).
         * @param dmask [in] (optional) the type of drawable. Default is DrawableNode::Physical.
         * @return a drawable node.
         */
        virtual DrawableNode::Ptr makeDrawable(const std::string& filename, int dmask=DrawableNode::Physical) = 0;

        /**
         * @brief Create a new scene camera.
         * @param name [in] name of the camera.
         * @return the new scene camera.
         */
        virtual rw::common::Ptr<SceneCamera> makeCamera(const std::string& name) = 0;

        // ************************ abstract interface - fuctionality has default implementation *******************

        /**
         * @brief Make a group node.
         * @param name [in] name of the node.
         * @return new group node.
         */
        virtual GroupNode::Ptr makeGroupNode(const std::string& name);

        /**
         * @brief Create a camera group.
         * @param name [in] name of the group.
         * @return a new camera group.
         */
        virtual rw::common::Ptr<CameraGroup> makeCameraGroup(const std::string& name);

        //virtual rw::common::Ptr<CameraGroup> getCameraGroup(int groupidx);

        /**
         * @brief Find a camera group.
         * @param name [in] name of the group.
         * @return the camera group if found, else NULL.
         */
        virtual rw::common::Ptr<CameraGroup> findCameraGroup(const std::string& name);

        /**
         * @brief Add a camera group.
         * @param cgroup [in] the group to add.
         */
        virtual void addCameraGroup(rw::common::Ptr<CameraGroup> cgroup);

        /**
         * @brief Remove a camera group.
         * @param cgroup [in] the group to remove.
         */
        virtual void removeCameraGroup(rw::common::Ptr<CameraGroup> cgroup);

        /**
         * @brief Remove a camera group.
         * @param name [in] the name of the group.
         */
        virtual void removeCameraGroup(const std::string& name);

        /**
         * @brief Get all camera groups.
         * @return a list of camera groups.
         */
        virtual std::list<rw::common::Ptr<CameraGroup> > getCameraGroups();

        /**
         * @brief Set the root of the scene graph.
         * @param node [in] the root node.
         */
        virtual void setRoot(GroupNode::Ptr node);

        /**
         * @brief Get the root of the scene graph.
         * @return the root group node.
         */
        virtual GroupNode::Ptr getRoot();

        /**
         * @brief add a drawable to a node
         */
        virtual void addChild(rw::common::Ptr<SceneNode> child, GroupNode::Ptr parent);

        /**
         * @brief get all drawables in the scene.
         * @return
         */
        virtual std::vector<DrawableNode::Ptr> getDrawables();

        /**
         * @brief get a vector of drawables attached to a node
         * @param node
         * @return
         */
        virtual std::vector<DrawableNode::Ptr> getDrawables(rw::common::Ptr<SceneNode> node);

        /**
         * @brief get all drawable nodes in the subtree of \b node. nodes of type camera will
         * not be traversed
         * @param node [in]
         * @return
         */
        virtual std::vector<DrawableNode::Ptr> getDrawablesRec(rw::common::Ptr<SceneNode> node);

        /**
         * @brief Find a drawable node in the scene graph.
         * @param name [in] the name of the drawable node.
         * @return the node if found, NULL otherwise.
         */
        virtual DrawableNode::Ptr findDrawable(const std::string& name);

        /**
         * @copydoc findDrawable(const std::string&)
         * @param node [in] search only this node and all children recursively.
         */
        virtual DrawableNode::Ptr findDrawable(const std::string& name, rw::common::Ptr<SceneNode> node);

        /**
         * @brief Find multiple drawable nodes in the scene graph.
         * @param name [in] the name of the drawable node(s).
         * @return a vector of all nodes with the given name.
         */
        virtual std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

        /**
         * @brief Remove a node.
         * @param node [in] the group node to remove.
         * @return true if removed, false otherwise.
         */
        virtual bool removeDrawables(GroupNode::Ptr node);

        /**
         * @brief Remove all drawables with a given name.
         * @param name [in] the name.
         * @return true if removed, false otherwise.
         */
        virtual bool removeDrawables(const std::string& name);

        /**
         * @brief Remove a specific drawable.
         * @param drawable [in] the drawable.
         * @return true if removed, false otherwise.
         */
        virtual bool removeDrawable(DrawableNode::Ptr drawable);

        /**
         * @brief Remove a specific drawable.
         * @param drawable [in] the drawable.
         * @param node [in] only search this node and children recursively.
         * @return true if removed, false otherwise.
         */
        virtual bool removeDrawable(DrawableNode::Ptr drawable, rw::common::Ptr<SceneNode> node);

        /**
         * @brief Remove a specific drawable with a given name.
         * @param name [in] the name.
         * @return true if removed, false otherwise.
         */
        virtual bool removeDrawable(const std::string& name);

		/**
		 * @brief Removes child with the specified name from the \b node.
		 *
		 * @param name [in] Name of child to remove
		 * @param node [in] Node to remove from
		 * @return true if found and successfully removed.
		 */
        virtual bool removeChild(const std::string& name, GroupNode::Ptr node);

        //! @brief Type of a visitor function. Returns true if visit is done.
        typedef boost::function<bool(rw::common::Ptr<SceneNode>& node, rw::common::Ptr<SceneNode>& parent)> NodeVisitor;

        //! @brief Type of a filter function.
        typedef boost::function<bool(const rw::common::Ptr<SceneNode>& node)> NodeFilter;

        /**
         * @brief Traverse all nodes in the subtree under \b node recursively.
         * @param node [in] the root node.
         * @param visitor [in] the visitor function to execute for each node visited.
         */
        void traverse(rw::common::Ptr<SceneNode>& node, NodeVisitor& visitor);

        /**
         * @copydoc traverse(rw::common::Ptr<SceneNode>&, NodeVisitor&)
         * @param postvisitor [in] the visitor function to execute for each node visited afterwards.
         */
        void traverse(rw::common::Ptr<SceneNode>& node, NodeVisitor& visitor, NodeVisitor& postvisitor);

        /**
         * @copydoc traverse(rw::common::Ptr<SceneNode>&, NodeVisitor&)
         * @param filter [in] filter which nodes to visit.
         */
        void traverse(rw::common::Ptr<SceneNode>& node, NodeVisitor& visitor, const NodeFilter& filter);

        /**
         * @copydoc traverse(rw::common::Ptr<SceneNode>&, NodeVisitor&, NodeVisitor&)
         * @param filter [in] filter which nodes to visit.
         */
        void traverse(rw::common::Ptr<SceneNode>& node, NodeVisitor& visitor, NodeVisitor& postvisitor, const NodeFilter& filter);

    protected:
        //! @brief Construct new scene graph with only a root node "Root".
        SceneGraph():_root(rw::common::ownedPtr(new GroupNode("Root"))){}

        /**
         * @brief Construct a new scene graph with a given root node.
         * @param root [in] the root node.
         */
        SceneGraph(GroupNode::Ptr root):_root(root){}

        //! @brief The root node of the graph.
        GroupNode::Ptr _root;

        //! @brief The camera groups in the graph.
        std::list<rw::common::Ptr<CameraGroup> > _cameraGroups;

    };

}
}

#endif /* RWSCENEGRAPH_HPP_ */
