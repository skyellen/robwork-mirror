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


#ifndef RWLIBS_OPENGL_RENDERARROW_HPP
#define RWLIBS_OPENGL_RENDERARROW_HPP

/**
 * @file RenderFrame.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include <rw/graphics/Render.hpp>

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
    /*@{*/

    /**
     * @brief RenderFrame makes a visualization of a frame
     */
    class RenderArrow : public rw::graphics::Render
    {
    private:
        float _size;
        mutable GLUquadricObj *_quadratic;
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderArrow> Ptr;

        /**
         * @brief Constructs a RenderFrame
         */
        RenderArrow():_size(1.0),
                _quadratic(NULL)
        {}

        /**
         * @brief Destructor
         */
        virtual ~RenderArrow(){
            if (_quadratic != NULL)
                gluDeleteQuadric(_quadratic);
        }

        /**
         * @brief Set the size.
         * @param size [in] size of the frame coordinate system
         */
        void setSize(float size){
            _size = size;
        }

    	/* Functions inherited from Render */

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const
        {
            if(_quadratic==NULL)
                _quadratic = gluNewQuadric();
            const float width = 0.3f;

            const float REL_WIDTH = 0.05f;
            const float lenHead = _size*3*REL_WIDTH; // the arrow head
            const float widthHead = _size*2*REL_WIDTH;
            const float lenBody = _size-lenHead; // the arrow body
            const float widthBody = width*_size*REL_WIDTH;

            // Nice frame
            // Draw z-axis
            glColor4f(0.0f, 0.0f, 1.0f, (float)alpha); // Blue color
            gluCylinder(_quadratic, widthBody, widthBody, lenBody, 32, 32);    // Draw Our Cylinder
            glTranslatef(0.0f,0.0f,lenBody);// Center The Cone
            gluCylinder(_quadratic,widthHead,0.0f,lenHead,32,32); // A Cone
        }

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
