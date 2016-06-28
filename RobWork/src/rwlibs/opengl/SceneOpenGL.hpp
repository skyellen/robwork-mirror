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
        //! @copydoc SceneGraph::draw
        void draw(rw::graphics::SceneGraph::RenderInfo& info);
        void draw(rw::graphics::SceneGraph::RenderInfo& info, rw::graphics::SceneNode::Ptr node);

        rw::graphics::DrawableNode::Ptr pickDrawable(rw::graphics::SceneGraph::RenderInfo& info, int x, int y);

        rw::math::Vector3D<> unproject(rw::graphics::SceneCamera::Ptr camera, int x, int y);


        void update();

        // interface for adding drawables
        //! @copydoc SceneGraph::addFrameAxis
        rw::graphics::DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask);
        //! @copydoc SceneGraph::addGeometry
        rw::graphics::DrawableGeometryNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::geometry::Geometry> geom, int dmask);
        //! @copydoc SceneGraph::addLines
        rw::graphics::DrawableGeometryNode::Ptr makeDrawable(const std::string& name, const std::vector<rw::geometry::Line >& lines, int dmask);

        //! @copydoc SceneGraph::makeDrawable
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::graphics::Model3D> model, int dmask);
        //! @copydoc SceneGraph::makeDrawable
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const rw::sensor::Image& img, int dmask);
        //! @copydoc SceneGraph::makeDrawable
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const rw::geometry::PointCloud& scan, int dmask);
        //! @copydoc SceneGraph::makeDrawable
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, const std::string& text);
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<rw::graphics::Render> render, int dmask);
        rw::graphics::DrawableNode::Ptr makeDrawable(const std::string& filename, int dmask);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::DrawableModelInfo& info);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::CollisionModelInfo& info);
        //rw::graphics::DrawableNode::Ptr makeDrawable(const rw::models::Object& info);

        rw::common::Ptr<rw::graphics::SceneCamera> makeCamera(const std::string& name);
        rw::common::Ptr<rw::graphics::CameraGroup> makeCameraGroup(const std::string& name);
        void clear();
    private:


    private:
        SceneOpenGL(const SceneOpenGL&);
        SceneOpenGL& operator=(const SceneOpenGL&);
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
