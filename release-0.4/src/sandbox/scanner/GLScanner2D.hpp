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


#ifndef RWLIBS_SIMULATION_GLSCANNER2D_HPP
#define RWLIBS_SIMULATION_GLSCANNER2D_HPP

#include <rw/sensor/Scanner2D.hpp>

namespace rwlibs { namespace simulation {

    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief
     */
    class GLScanner2D: public rw::sensor::Scanner2D {
    public:
        /**
         * @brief constructor
         */
        GLScanner2D();

        virtual ~GLScanner2D();

    };

    /* @} */

}} // end namespaces

#endif // end include guard
