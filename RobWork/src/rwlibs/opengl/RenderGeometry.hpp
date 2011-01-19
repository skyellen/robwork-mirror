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

#ifndef RWLIBS_OPENGL_RENDERGEOMETRY_HPP
#define RWLIBS_OPENGL_RENDERGEOMETRY_HPP

//! @file RenderGeometry.hpp

#include <rw/graphics/Render.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/TriMesh.hpp>

namespace rwlibs {
namespace opengl {

	//! @addtogroup drawable
	// @{


    /**
     * @brief RenderGeometry provide a class for visualizing Geometry objects.
     */
    class RenderGeometry: public rw::graphics::Render {
    private:
    	mutable GLfloat _diffuse[4];
    	GLfloat _ambient[4];
    	GLfloat _emission[4];
    	GLfloat _specular[4];
    	GLfloat _shininess[1];

    	GLuint _displayListId;
		rw::geometry::Geometry::Ptr _geometry;
        float _r, _g, _b;

    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderGeometry> Ptr;

        /**
         * @brief Constructs RenderGeometry object
         *
         * Constructs a RenderGeometry object to visualize the geometry.
         * The RenderGeometry takes ownership of the Geometry object and
         * deletes when done with it.
         *
         * @param geo [in] the geometry to draw
         */
		RenderGeometry(rw::geometry::Geometry::Ptr geo);

        /**
         * @brief Constructs RenderGeometry object
         *
         * Constructs a RenderGeometry object to visualize the geometry.
         * The RenderGeometry takes ownership of the Geometry object and
         * deletes when done with it.
         *
         * @param geo [in] the geometry to draw
         */
        RenderGeometry(rw::geometry::TriMesh::Ptr mesh);

        /**
         * @brief Destructor
         */
        virtual ~RenderGeometry();

        /**
         * @brief Sets color of the object
         * @param r [in] red color component
         * @param g [in] green color component
         * @param b [in] blue color component
         */
        void setColor(float r, float g, float b);

        /**
         * @brief set a new geometry on this render
         * @param geom [in] geometry that is to be rendered
         */
		void setGeometry(rw::geometry::Geometry::Ptr geom);

        /**
         * @copydoc Render::draw
         */
        void draw(rw::graphics::DrawableNode::DrawType type, double alpha) const;

    };

#ifdef RW_USE_DEPRECATED
	/**
	 * @brief Definition of smartpointer to RenderGeometry
	 */
	typedef rw::common::Ptr<RenderGeometry> RenderGeometryPtr;
#endif
    //! @}

} //end namespace opengl
} //end namespace rwlibs

#endif //#ifndef RWLIBS_OPENGL_RENDERGEOMETRY_HPP
