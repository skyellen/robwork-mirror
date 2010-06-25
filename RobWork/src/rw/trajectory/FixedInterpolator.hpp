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


#ifndef RW_TRAJECTORY_FIXEDINTERPOLATOR_HPP
#define RW_TRAJECTORY_FIXEDINTERPOLATOR_HPP

#include "Interpolator.hpp"

/**
   @file Interpolator.hpp
*/

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/


/**
 * @brief Implements a fixed value interpolator.
 *
 * The FixedInterpolator will always return the same value and 0 for velocity and acceleration.
 */
template <class T>
class FixedInterpolator: public Interpolator<T> 
{
public:
    /**
     * @brief Constructs a FixedInterpolator with value \b value and duration \b duration.
     *
     * It is assumed that the template type T can be zeroed by
     * \code
     *       for (size_t i = 0; i < _zeroValue.size(); i++)
     *           _zeroValue[i] = 0;
     * \endcode
     * to get proper return values for velocity and acceleration.
     *
     * @param value [in] Value to return for x(double t).
     * @param duration [in] Duration of the interpolator.
     */
    FixedInterpolator(const T& value, double duration):
      _value(value),
      _zeroValue(value),
      _duration(duration)
    {
        for (size_t i = 0; i < _zeroValue.size(); i++)
            _zeroValue[i] = 0;
    }

    /**
     * @brief Constructs a FixedInterpolator with value \b value, duration \b duration and
     * returning \b zeroValue for velocity and acceleration.
     *
     * @param value [in] Value to return for x(double t).
     * @param duration [in] Duration of the interpolator.
     */
    FixedInterpolator(const T& value, const T& zeroValue, double duration):
        _value(value),
        _zeroValue(zeroValue),
        _duration(duration)
    {
        
    }

    /**
     * @copydoc Interpolator::x(double)
     */
    T x(double t) const { return _value; }

    /**
     * @copydoc Interpolator::dx(double)
     */
    T dx(double t) const { return _zeroValue; }

    /**
     * @copydoc Interpolator::ddx(double)
     */
    T ddx(double t) const { return _zeroValue; }

    /**
     * @copydoc Interpolator::duration()
     */
    double duration() const { return _duration; }

private:
    T _value;
    T _zeroValue;
    double _duration;


};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif //#ifndef RW_TRAJECTORY_FIXEDINTERPOLATOR_HPP
