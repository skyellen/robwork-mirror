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


#ifndef rw_interpolator_LloydHaySegment_HPP
#define rw_interpolator_LloydHaySegment_HPP

/**
 * @file LloydHaySegment.hpp
 */

#include "FunctionSegment.hpp"

#include <rw/math/Q.hpp>
namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class represents a Lloyd Hayward blended segment.
     *
     */
    class LloydHaySegment : public FunctionSegment
    {
    private:
        const FunctionSegment *_x1,*_x2;
        double _k;
        double _tau;
        //double _length; now part of FunctionSegment

    public:
        /**
         * @brief Constructs a Lloyd Hayward blend between two FunctionSegments. The end of x1 and
         * start of x2 is required to be the same. Also the dimension of x1 should equal the dimension
         * of x2.
         * @param x1 [in] The first of the two segments.
         * @param x2 [in] The second of the two segments.
         * @param k [in] A parameter that controls the amount of acceleration compensation to apply.
         * @param tau [in] The half of the blend time.         
         */
        LloydHaySegment(const FunctionSegment& x1, const FunctionSegment& x2, double k, double tau);

        /**
         * @brief Destructor
         */
        virtual ~LloydHaySegment(){};

        /**
         * @copydoc FunctionSegment::getX
         */
        rw::math::Q getX(double t) const;

        /**
         * @copydoc FunctionSegment::getXd
         */
        rw::math::Q getXd(double t) const;

        /**
         * @copydoc FunctionSegment::getXdd
         */
        rw::math::Q getXdd(double t) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
