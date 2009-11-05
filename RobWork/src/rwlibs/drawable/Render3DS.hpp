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


#ifndef rwlibs_drawable_Render3DS_HPP
#define rwlibs_drawable_Render3DS_HPP

/**
 * @file Render3DS.hpp
 */

#include "Model_3DS.h"
#include "Render.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cstring>
#include <iostream>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads 3d scenes or objects from a 3ds file
     * format.
     *
     */
    class Render3DS : public Render {
    private:
        mutable Model_3DS _model;

    public:
        /**
         * @brief creates a Render3DS given a 3DS file.
         * @param filename [in] - the path and name of the 3DS file
         */
        Render3DS(const std::string &filename);

        /**
         * @brief Destructor
         */
    	virtual ~Render3DS(){}

    	// Functions inherited from Render

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
