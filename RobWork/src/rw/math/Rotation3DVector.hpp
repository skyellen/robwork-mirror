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


#ifndef RW_MATH_ROTATION3DVECTOR_HPP
#define RW_MATH_ROTATION3DVECTOR_HPP

/**
 * @file Rotation3DVector.hpp
 */


#include "Rotation3D.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief An abstract base class for Rotation3D parameterisations
     *
     * Classes that represents a parametrisation of a 3D rotation may inherit
     * from this class
     */
    template<class T = double>
    class Rotation3DVector{
    public:
        /**
         * @brief Virtual destructor
         */
        virtual ~Rotation3DVector(){}

        /**
         * @brief Returns the corresponding @f$ 3\times 3 @f$ Rotation matrix
         * @return The rotation matrix
         */
        virtual const Rotation3D<T> toRotation3D() const = 0;

    protected:
        /**
         * @brief Copy Constructor
         *
         * We allow subclasses of this class to be copied.
         */
        Rotation3DVector(const Rotation3DVector&) {}

        /**
         * @brief Assignment operator is protected to force subclasses to
         * implement it by themself.
         */
        Rotation3DVector& operator=(const Rotation3DVector&) { return *this;}

    protected:
        /**
         * @brief Default Constructor
         */
        Rotation3DVector() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
