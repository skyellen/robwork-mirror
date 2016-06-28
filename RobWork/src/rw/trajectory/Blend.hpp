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


#ifndef RW_TRAJECTORY_BLEND_HPP
#define RW_TRAJECTORY_BLEND_HPP

/**
 * @file Blend.hpp
 */

#include <rw/common/Ptr.hpp>

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/

/**
 * @brief Interface for blending
 *
 * A Blend describes a way to blend between to consecutive interpolators. If we let
 * \f$t_1\f$ be the time switching from one interpolator to the next, then the blend
 * control the path in the interval \f$[t_1-\tau_1;t_1+\tau_2]\f$.
 *
 * See the specific implementations for at description of which template
 * arguments that are valid.
 */
template <class T>
class Blend
{
public:
	//! @brief smart pointer type to this class
	typedef typename rw::common::Ptr<Blend> Ptr;

    /**
     * @brief Destructor
     */
    virtual ~Blend() {}

    /**
     * @brief The position for a given time t
     * @param t [in] \f$t\in[0,\tau_1+\tau_2] \f$
     * @return Position at time \b t
     */
    virtual T x(double t) const = 0;

    /**
     * @brief The velocity for a given time t
     * @param t [in] \f$t\in[0,\tau_1+\tau_2] \f$
     * @return Velocity at time \b t
     */
    virtual T dx(double t) const = 0;

    /**
     * @brief The acceleration for a given time t
     * @param t [in] \f$t\in[0,\tau_1+\tau_2] \f$
     * @return Acceleration at time \b t
     */
    virtual T ddx(double t) const = 0;

    /**
     * @brief The time \f$\tau_1\f$ as defined in class definition
     * @return \f$\tau_1\f$
     */
    virtual double tau1() const = 0;

    /**
     * @brief The time \f$\tau_2\f$ as defined in class definition
     * @return \f$\tau_2\f$
     */
    virtual double tau2() const = 0;

};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif //RW_TRAJECTORY_BLEND_HPP
