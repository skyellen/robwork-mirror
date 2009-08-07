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


#ifndef RWLIBS_DRAWABLE_RENDERFRAME_HPP
#define RWLIBS_DRAWABLE_RENDERFRAME_HPP

/**
 * @file RenderFrame.hpp
 */

#include <rwlibs/os/rwgl.hpp>

#include "Render.hpp"

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief RenderFrame makes a visualization of a frame
     */
    class RenderFrame : public Render
    {
    private:
        float _size;
        GLUquadricObj *_quadratic;
        GLuint _displayListId;

        mutable float _green[4];
        mutable float _red[4];
        mutable float _blue[4];
    public:

        /**
         * @brief Constructs a RenderFrame
         * @param size [in] size of the frame coordinate system
         */
        RenderFrame(float size=1);

        /**
         * @brief Destructor
         */
        virtual ~RenderFrame(){
        	glDeleteLists(_displayListId,1);
        };

    	/* Functions inherited from Render */

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
