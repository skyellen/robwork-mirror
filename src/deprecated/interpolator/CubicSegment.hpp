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


#ifndef rw_interpolator_CubicSegment_HPP
#define rw_interpolator_CubicSegment_HPP

/**
 * @file CubicSegment.hpp
 */

#include "FunctionSegment.hpp"
#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class represents a 3-degree polynomial function, used
     * in Cubic Splines hence the name CubicSegment.
     *
     * \f$ \bf{f}(t)= \bf{a} + \bf{b}\cdot t + \bf{c}\cdot t^2 \bf{d}\cdot t^3 \f$
     */
    class CubicSegment : public FunctionSegment
    {
    private:
        /**
         * @brief parameters in the cubic spline function
         */
        rw::math::Q _a,_b,_c,_d;

    public:
        /**
         * @brief Contructor
         *
         * @param a [in] the a parameter in the 3-degree polynomial
         * @param b [in] the b parameter in the 3-degree polynomial
         * @param c [in] the c parameter in the 3-degree polynomial
         * @param d [in] the d parameter in the 3-degree polynomial
         *
         * @param length [in] the length of the interval
         */
        CubicSegment(const rw::math::Q& a,
                     const rw::math::Q& b,
                     const rw::math::Q& c,
                     const rw::math::Q& d,
                     double length):
            FunctionSegment(length),
            _a(a),_b(b),_c(c),_d(d)
            
        {
            if (length <= 0)
                RW_THROW("Length of segment must be positive");
        }

  
        /**
         * @brief Deconstructor
         */
        virtual ~CubicSegment(){}

        /**
         * @brief This function enable updating the internal parameters of the cubic
         * spline segment.
         * @param a [in] the a parameter in the 3-degree polynomial
         * @param b [in] the b parameter in the 3-degree polynomial
         * @param c [in] the c parameter in the 3-degree polynomial
         * @param d [in] the d parameter in the 3-degree polynomial
         * @param length [in] the length of the segment
         */
        void setParameters(
            const rw::math::Q& a,
            const rw::math::Q& b,
            const rw::math::Q& c,
            const rw::math::Q& d,
            double length)
        {
            _a=a; _b=b; _c=c; _d=d;
            if (_length <= 0) 
                RW_THROW("Length of segment must be a positive");
            
            _length = length;
        
        }

        /**
         * @copydoc FunctionSegment::getX
         *
         * @note The cubic polynomial is given by a 3-degree polynomial:
         * \f$ \bf{f}(t)= \bf{a} + \bf{b}\cdot t + \bf{c}\cdot t^2 \bf{d}\cdot t^3 \f$
         */
        rw::math::Q getX(double t) const
        {            
            t /= _length;
            double tpow2 = t*t;
            double tpow3 = tpow2*t;
            return _a+_b*t+_c*tpow2+_d*tpow3;
        }

        /**
         * @copydoc FunctionSegment::getXd
         *
         * @note The derivative is a 2-degree polynomial:
         * \f$ \bf{df}(t)= \bf{b} + 2\cdot \bf{c}\cdot t + 3\cdot \bf{d}\cdot t^2 \f$
         */
        rw::math::Q getXd(double t) const
        {
            t /= _length;
            return _b + 2*_c*t + 3*_d*t*t;
        }

        /**
         * @copydoc FunctionSegment::getXdd
         *
         * @note The second derivative is a 1-degree polynomial:
         * \f$ \bf{df}(t)= 2\cdot \bf{c} + 6\cdot \bf{d}\cdot t \f$
         */
        rw::math::Q getXdd(double t) const
        {
            t /= _length;
            return 2*_c + 6*_d*t;
        }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
