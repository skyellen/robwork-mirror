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

#include <rw/sensor/Image.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/graphics/Render.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <rw/graphics/DrawableGeometryNode.hpp>

#include "SceneCamera.hpp"
#include "SceneNode.hpp"
#include "GroupNode.hpp"

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
     *
     *
     *
     *
     */
    class SceneGraph {
    public:
        //! @brief smar pointer type of this class
        typedef rw::common::Ptr<SceneGraph> Ptr;

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
            RenderInfo()
                    //:cameraGroup(0)
            {}
            //int cameraGroup;
            CameraGroup::Ptr cams;
            //DrawType dtype;
        };

        virtual ~SceneGraph(){};

        /**
         * @brief draws the scene, using the specified render information
         */
        virtual void draw(RenderInfo& info) = 0;

        /**
         * @brief picks the drawable in the scene that intersects with the ray (x,y,-1) in
         * camera coordinates.
         *
         *
         * @param info
         * @return
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
        virtual void clear() = 0;

        // interface for adding drawables
        virtual DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask=DrawableNode::Physical)= 0;
        virtual DrawableGeometryNode::Ptr makeDrawable(const std::string& name, rw::geometry::Geometry::Ptr geom, int dmask=DrawableNode::Physical) = 0;
        virtual DrawableGeometryNode::Ptr makeDrawable(const std::string& name, const std::vector<rw::geometry::Line >& lines, int dmask=DrawableNode::Physical)= 0;

        virtual DrawableNode::Ptr makeDrawable(const std::string& name, const rw::sensor::Image& img, int dmask=DrawableNode::Virtual)= 0;

        virtual DrawableNode::Ptr makeDrawable(const std::string& name, const rw::geometry::PointCloud& scan, int dmask=DrawableNode::Virtual)= 0;

        virtual DrawableNode::Ptr makeDrawable(const std::string& name, Model3D::Ptr model, int dmask=DrawableNode::Physical) = 0;
        virtual DrawableNode::Ptr makeDrawable(const std::string& name, rw::graphics::Render::Ptr render, int dmask=DrawableNode::Physical)= 0;
        //virtual DrawableNode::Ptr makeDrawable(const rw::models::DrawableModelInfo& info) = 0;
        //virtual DrawableNode::Ptr makeDrawable(const rw::models::CollisionModelInfo& info) = 0;
        virtual DrawableNode::Ptr makeDrawable(const std::string& filename, int dmask=DrawableNode::Physical) = 0;
        virtual rw::common::Ptr<SceneCamera> makeCamera(const std::string& name) = 0;

        /// ************************ abstract interface - fuctionality has default implementation *******************

        virtual GroupNode::Ptr makeGroupNode(const std::string& name);

        virtual rw::common::Ptr<CameraGroup> makeCameraGroup(const std::string& name);
        //virtual rw::common::Ptr<CameraGroup> getCameraGroup(int groupidx);
        virtual rw::common::Ptr<CameraGroup> findCameraGroup(const std::string& name);
        virtual void addCameraGroup(rw::common::Ptr<CameraGroup> cgroup);
        virtual void removeCameraGroup(rw::common::Ptr<CameraGroup> cgroup);
        virtual void removeCameraGroup(const std::string& name);
        virtual std::list<rw::common::Ptr<CameraGroup> > getCameraGroups();

        virtual void setRoot(GroupNode::Ptr node);
        virtual GroupNode::Ptr getRoot();

        /**
         * @brief add a drawable to a node
         */
        virtual void addChild(SceneNode::Ptr child, GroupNode::Ptr parent);

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
        virtual std::vector<DrawableNode::Ptr> getDrawables(SceneNode::Ptr node);

        /**
         * @brief get all drawable nodes in the subtree of \b node. nodes of type camera will
         * not be traversed
         * @param node [in]
         * @return
         */
        virtual std::vector<DrawableNode::Ptr> getDrawablesRec(SceneNode::Ptr node);

        virtual DrawableNode::Ptr findDrawable(const std::string& name);
        virtual DrawableNode::Ptr findDrawable(const std::string& name, SceneNode::Ptr node);
        virtual std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

        virtual bool removeDrawables(GroupNode::Ptr node);
        virtual bool removeDrawables(const std::string& name);
        virtual bool removeDrawable(DrawableNode::Ptr drawable);
        virtual bool removeDrawable(DrawableNode::Ptr drawable, SceneNode::Ptr node);
        virtual bool removeDrawable(const std::string& name);
		/**
		 * @brief Removes child with the specified name from the \b node.
		 *
		 * @param name [in] Name of child to remove
		 * @param node [in] Node to remove from
		 * @return true if found and successfully removed.
		 */
        virtual bool removeChild(const std::string& name, GroupNode::Ptr node);

        //! returns true if visit is done
        typedef boost::function<bool(SceneNode::Ptr& node, SceneNode::Ptr& parent)> NodeVisitor;
        typedef boost::function<bool(const SceneNode::Ptr& node)> NodeFilter;

        void traverse(SceneNode::Ptr& node, NodeVisitor& visitor);
        void traverse(SceneNode::Ptr& node, NodeVisitor& visitor, NodeVisitor& postvisitor);
        void traverse(SceneNode::Ptr& node, NodeVisitor& visitor, const NodeFilter& filter);
        void traverse(SceneNode::Ptr& node, NodeVisitor& visitor, NodeVisitor& postvisitor, const NodeFilter& filter);

    protected:
        SceneGraph():_root(rw::common::ownedPtr(new GroupNode("Root"))){}
        SceneGraph(GroupNode::Ptr root):_root(root){}
        GroupNode::Ptr _root;
        std::list<rw::common::Ptr<CameraGroup> > _cameraGroups;

    };

}
}

#endif /* RWSCENEGRAPH_HPP_ */
