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


#ifndef rw_geometry_GeometrySTL_HPP
#define rw_geometry_GeometrySTL_HPP

/**
 * @file GeometrySTL.hpp
 */

#include "Face.hpp"

namespace rw { namespace geometry {
    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief This class loads in STL geometry objects
     */

    class GeometrySTL {
    public:
        /**
         * @brief loads a stl file. Automaticly detects if the STL format is
         * ASCII or Binary
         *
         * @param filename [in] path and name of file to load
         *
         * @param result [out] vector into which the result is stored
         */
        static void load(const std::string &filename,
                         std::vector<Face<float> >& result);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
