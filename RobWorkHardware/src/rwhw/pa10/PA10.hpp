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

#ifndef RWHW_PA10_HPP
#define RWHW_PA10_HPP

/**
 * @file PA10.hpp
 */

#include <rw/math/Q.hpp>

namespace rwhw {

    /** @addtogroup pa10 */
    /*@{*/

    /**
     * @brief Interface for Mitsubishi PA10 driver and virtual device
     */
    class PA10 {
    public:
        /**
         * @brief destructor
         */
        virtual ~PA10() {};

        /**
         * @brief Connect to a releases brakes of PA10
         *
         * Connects to the PA10 and releases the brakes.
         *
         * After start is successfully called, the user will
         * have to call update no less than every 400ms.
         * otherwise the robot stops.
         *
         * @param success [out] whether the devices was successfully started
         * @return the current configuration
         */
        virtual rw::math::Q start(bool& success) = 0;

        /**
         * @brief Sets the current threads I/O privileges
         *
         * If the thread calling update is different from the one calling
         * start it is necessary to call initializeThread to set up the
         * permissions.
         */
        virtual void initializeThread() = 0;

        /**
         * @brief Updates the velocity of the robot and returns
         * the current configuration.
         * @param dq [in] desired joint velocity
         * @return the current joint configuration
         */
        virtual rw::math::Q update(const rw::math::Q& dq) = 0;

        /**
         * @brief Stops the PA10 and lock brakes
         */
        virtual void stop() = 0;

    protected:
        /**
         * @brief Creates object
         */
        PA10() {}

    private:
        PA10(const PA10&);
        PA10& operator=(const PA10&);
    };

    /**@}*/
} // end namespaces

#endif // end include guard
