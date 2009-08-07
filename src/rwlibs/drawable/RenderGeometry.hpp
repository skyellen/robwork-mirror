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


#ifndef RWLIBS_DRAWABLE_RENDERGEOMETRY_HPP
#define RWLIBS_DRAWABLE_RENDERGEOMETRY_HPP


#include "Render.hpp"

#include <rw/geometry/Geometry.hpp>

namespace rwlibs {
namespace drawable {

    /**
     * @brief RenderGeometry provide a class for visualizing Geometry objects
     *
     */
    class RenderGeometry: public Render {
    private:
    	mutable GLfloat _diffuse[4];
    	GLfloat _ambient[4];
    	GLfloat _emission[4];
    	GLfloat _specular[4];
    	GLfloat _shininess[1];

    	GLuint _displayListId;
        rw::geometry::Geometry* _geometry;
        float _r, _g, _b;

    public:
        /**
         * @brief Constructs RenderGeometry object
         *
         * Constructs a RenderGeometry object to visualize the geometry.
         * The RenderGeometry takes ownership of the Geometry object and
         * deletes when done with it.
         *
         * @param geo [in] the geometry to draw
         */
        RenderGeometry(rw::geometry::Geometry* geo);

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
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

        void setGeometry(rw::geometry::Geometry* geom);


    };

} //end namespace drawable
} //end namespace rwlibs

#endif //#ifndef RWLIBS_DRAWABLE_RENDERGEOMETRY_HPP
