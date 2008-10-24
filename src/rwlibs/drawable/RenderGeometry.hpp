/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
