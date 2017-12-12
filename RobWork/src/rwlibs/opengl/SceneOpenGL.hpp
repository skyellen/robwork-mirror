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

#ifndef RWLIBS_OPENGL_SCENEOPENGL_HPP
#define RWLIBS_OPENGL_SCENEOPENGL_HPP

/**
 * @file SceneOpenGL.hpp
 */

#include <rw/graphics/SceneGraph.hpp>

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
    /*@{*/


    /**
     * @brief Helps with Visualizing a Workcell. SceneOpenGL is OpenGL specific
     */
    class SceneOpenGL: public rw::graphics::SceneGraph {
    public:
    	//! @brief Smart pointer type for SceneOpenGL.
        typedef rw::common::Ptr<SceneOpenGL> Ptr;

        /**
         * @brief Creates object
         */
        SceneOpenGL();

        /**
         * @brief Destroys object
         */
        virtual ~SceneOpenGL();

        /**
         * @brief Clears the drawable cache by deleting all drawables
         */
        void clearCache();

        // here comes utility functions for adding drawables to the scene INHERITED BY SceneGraph
        //! @copydoc rw::graphics::SceneGraph::draw
        void draw(rw::graphics::SceneGraph::RenderInfo& info);
        /**
         * @copydoc rw::graphics::SceneGraph::draw(rw::graphics::SceneGraph::RenderInfo&, rw::common::Ptr<rw::graphics::SceneNode>)
         * @param node [in] draw only this subtree.
         */
        void draw(rw::graphics::SceneGraph::RenderInfo& info, rw::graphics::SceneNode::Ptr node);

        //! @copydoc rw::graphics::SceneGraph::pickDrawable
        rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);

        //! @copydoc rw::graphics::SceneGraph::unproject
        rw::math::Vector3D<> unproject(rw::graphics::SceneCamera::Ptr camera, int x, int y);

        //! @copydoc rw::graphics::SceneGraph::update
        void update();

        // interface for adding drawables
        //! @copydoc rw::graphics::SceneGraph::makeDrawableFrameAxis
        rw::graphics::DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,rw::common::Ptr<rw::geometry::Geometry>,int)
        rw::graphics::DrawableGeometryNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::geometry::Geometry> geom, int dmask);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,const std::vector<rw::geometry::Line >&,int)
        rw::graphics::DrawableGeometryNode::Ptr makeDrawable(const std::string& name, const std::vector<rw::geometry::Line >& lines, int dmask);

        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,rw::common::Ptr<rw::graphics::Model3D>,int)
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::graphics::Model3D> model, int dmask);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,const class rw::sensor::Image&,int)
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const class rw::sensor::Image& img, int dmask);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,const rw::geometry::PointCloud&,int)
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const rw::geometry::PointCloud& scan, int dmask);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const std::string& text);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,rw::common::Ptr<rw::graphics::Render>,int)
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::graphics::Render> render, int dmask);
        //! @copydoc rw::graphics::SceneGraph::makeDrawable(const std::string&,int)
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& filename, int dmask);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::DrawableModelInfo& info);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::CollisionModelInfo& info);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::Object& info);

        //! @copydoc rw::graphics::SceneGraph::makeCamera
        rw::common::Ptr<rw::graphics::SceneCamera> makeCamera(const std::string& name);
        //! @copydoc rw::graphics::SceneGraph::makeCameraGroup
        rw::common::Ptr<rw::graphics::CameraGroup> makeCameraGroup(const std::string& name);
        void clear();

    protected:
        static std::string toString(const GLenum error);

    private:
        SceneOpenGL(const SceneOpenGL&);
        SceneOpenGL& operator=(const SceneOpenGL&);
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
