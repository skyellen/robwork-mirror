/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef rwlibs_drawable_DrawableUtil_HPP
#define rwlibs_drawable_DrawableUtil_HPP

/**
 * @file DrawableUtil.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rwlibs/os/rwgl.hpp>

#include <boost/shared_ptr.hpp>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Utility class for drawable stuff
     */
    class DrawableUtil
    {
    public:

    	/**
    	 * @brief copy a RW Transform3D to a GL transform representation
    	 * @param transform [in] the Transform3D object
    	 * @param gltrans [in] a GLfloat array of size 16
    	 */
        static void transform3DToGLTransform(
            const rw::math::Transform3D<>& transform,
            GLfloat* gltrans)
        {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++)
                    gltrans[j + 4 * k] =
                        (float)transform(j,k);

                gltrans[12 + j] =
                    (float)transform(j, 3);
            }

            gltrans[3] = gltrans[7] = gltrans[11] = 0;
            gltrans[15] = 1;
        }

        static void drawGLVertex(const rw::math::Vector3D<>& v){
            glVertex3f(float(v(0)), float(v(1)), float(v(2)));    // Bottom Left
        }


        static void setupHighlightLight() {
            //We set up GL_LIGHT7 for highlighting
            GLfloat light7_ambient[] =  {1.0f, 0.0f, 0.0f, 1.0f};
            GLfloat light7_diffuse[] =  {.6f, .3f, 0.3f, 1.0f};
            GLfloat light7_specular[] = { 0.5, 0.2, 0.2, 1.0f};
            GLfloat light7_position[] = {.5f, .5f, 1.0f, 0.0f};

            glLightfv(GL_LIGHT7, GL_AMBIENT, light7_ambient);
            glLightfv(GL_LIGHT7, GL_DIFFUSE, light7_diffuse);
            glLightfv(GL_LIGHT7, GL_SPECULAR, light7_specular);
            glLightfv(GL_LIGHT7, GL_POSITION, light7_position);

        }

    };

	/* @} */
}} // end namespaces

#endif // end include guard
