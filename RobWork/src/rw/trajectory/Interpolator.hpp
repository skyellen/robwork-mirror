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


#ifndef RW_TRAJECTORY_INTERPOLATOR_HPP
#define RW_TRAJECTORY_INTERPOLATOR_HPP

#include <rw/common/Ptr.hpp>

/**
   @file Interpolator.hpp
*/

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Interface for interpolators
     *
     * See the specific implementations for more details
     */
    template <class T>
    class Interpolator
    {
    public:
		//! @brief smart pointer type to this class
		typedef typename rw::common::Ptr<Interpolator> Ptr;
        /**
         * @brief Virtual destructor
         */
        virtual ~Interpolator() {}

        /**
         * @brief Position at time t
         * @param t [in] time between \b 0 and \b length
         * @return Position
         */
        virtual T x(double t) const = 0;

        /**
         * @brief Velocity at time t
         * @param t [in] time between \b 0 and \b length
         * @return Velocity
         */
        virtual T dx(double t) const = 0;

        /**
         * @brief Acceleration at time t
         * @param t [in] time between \b 0 and \b length
         * @return Acceleration
         */
        virtual T ddx(double t) const = 0;

        /**
         * @brief Returns the duration of the interpolator
         *
         * The duration is defined as the time it takes to move from one end
         * of the interpolator to the other.
         * @return duration
         */
        virtual double duration() const = 0;
    };

    /* @} */
}} // end namespaces
#endif // end include guard
