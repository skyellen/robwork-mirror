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


#ifndef RWLIBS_DRAWABLE_RENDER_HPP
#define RWLIBS_DRAWABLE_RENDER_HPP

/**
 * @file Render.hpp
 */

#include <rwlibs/os/rwgl.hpp>
#include <rw/common/Ptr.hpp>
#include <iostream>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Abstract base class for all renderable classes
     *
     * Classes that are able to render them self, may inherit from this class.
     */
    class Render {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<Render> Ptr;

        /**
         * @brief when calling render on the draw mode or type
         * can be specified.
         */
        enum DrawType {
            SOLID, //! Render in solid
            WIRE, //! Render in wireframe
            OUTLINE //! Render both solid and wireframe
        };

        /**
         * @brief destructor
         */
        virtual ~Render(){};

        /**
         * @brief draws the object.
         */
        virtual void draw(DrawType type, double alpha) const = 0;

    protected:
    	Render(){};

    private:
        Render(const Render&);
        Render& operator=(const Render&);
    };

    typedef rw::common::Ptr<Render> RenderPtr;

    /*@}*/
}} // end namespaces

#endif // end include guard
