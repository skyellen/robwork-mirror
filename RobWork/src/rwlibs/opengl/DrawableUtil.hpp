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


#ifndef RWLIBS_OPENGL_DrawableUtil_HPP
#define RWLIBS_OPENGL_DrawableUtil_HPP

/**
 * @file DrawableUtil.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rwlibs/os/rwgl.hpp>

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
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
            const rw::math::Transform3D<float>& transform,
            GLfloat* gltrans);

        /**
         * @brief copy a RW Transform3D to a GL transform representation
         * @param transform [in] the Transform3D object
         * @param gltrans [in] a GLfloat array of size 16
         */
        static void transform3DToGLTransform(
            const rw::math::Transform3D<double>& transform,
            GLfloat* gltrans);

        /**
         * @brief copy a RW Transform3D to a GL transform representation
         * @param transform [in] the Transform3D object
         * @param gltrans [in] a GLfloat array of size 16
         */
        static void transform3DToGLTransform(
            const rw::math::Transform3D<double>& transform,
            GLdouble* gltrans);

        /**
         * @brief multiplies the transform on the gl stack with the rw transform b transform
         * @param transform [in] the Transform3D object
         */
        static void multGLTransform(const rw::math::Transform3D<float>& transform)
        {
            GLfloat gltrans[16];
            transform3DToGLTransform(transform,gltrans);
            glMultMatrixf(gltrans);
        }

        /**
         * @brief multiplies the transform on the gl stack with the rw transform b transform
         * @param transform [in] the Transform3D object
         */
        static void multGLTransform(const rw::math::Transform3D<double>& transform)
        {
            GLfloat gltrans[16];
            transform3DToGLTransform(transform,gltrans);
            glMultMatrixf(gltrans);
        }

        /**
         * @brief draw a glVertex3
         * @param v [in] the Vector3D object
         */
        static void drawGLVertex(const rw::math::Vector3D<>& v){
            glVertex3f(float(v(0)), float(v(1)), float(v(2)));    // Bottom Left
        }

        /**
         * @brief sets up the highlighting functionality. Highlighting is enabled by enabling
         * and disabling GL_LIGHT7 when drawing objects.
         */
        static void setupHighlightLight() {
            //We set up GL_LIGHT7 for highlighting
            GLfloat light7_ambient[] =  {1.0f, 0.0f, 0.0f, 1.0f};
            GLfloat light7_diffuse[] =  {.6f, .3f, 0.3f, 1.0f};
            GLfloat light7_specular[] = { 0.5f, 0.2f, 0.2f, 1.0f};
            // directional assumes that we are looking from above
            GLfloat light7_position[] = {0.0f, 0.0f, -1.0f, 0.0f};

            glLightfv(GL_LIGHT7, GL_AMBIENT, light7_ambient);
            glLightfv(GL_LIGHT7, GL_DIFFUSE, light7_diffuse);
            glLightfv(GL_LIGHT7, GL_SPECULAR, light7_specular);
            glLightfv(GL_LIGHT7, GL_POSITION, light7_position);
        }

    };

	/* @} */
}} // end namespaces

#endif // end include guard
