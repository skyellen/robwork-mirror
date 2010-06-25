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

#ifndef RWHW_CRSA465_HPP
#define RWHW_CRSA465_HPP

/**
 * @file CRSA465.hpp
 */

#include <list>
#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include "crs_aci.hpp"

namespace rwhw {

    /** @addtogroup crrA465 */
    /* @{ */

    /**
     * @brief Device for controlling CRSA465
     */
    class CRSA465
    {
    public:
        /**
         * @brief Construct CRSA465 Device
         */
        CRSA465();

        /**
         * @brief Destructor of the Device
         * Once called the destructor makes sure the connection to the device
         * is correctly closed.
         */
        ~CRSA465();

        /**
         * @brief Connects to the CRS robot
         * @param port [in] string identifier of the serial port
         * @return whether the connection was successful
         */
        bool connect(const std::string& port);

        /**
         * @brief Disconnects from CRS robot
         */
        void disconnect();

        /**
         * @brief Tells whether the CRS robot is connected
         * @return connection status
         */
        bool isConnected() const;

        /**
         * @brief Sends home command to CRS robot
         */
        void home();

        /**
         * @brief Sends Cartesian move command to the robot
         * @param baseTtool [in] the desired base to tool transformation
         */
        void move(rw::math::Transform3D<>& baseTtool);

        /**
         * @brief Returns the base to tool transform.
         *
         * This method queries the robot for its base to tool transform.
         *
         * @return base to tool transform
         */
        rw::math::Transform3D<> getBaseTtool();

        /**
         * @brief Send joint move command to the robot
         * @param q [in] the desired joint position
         */
        void move(rw::math::Q& q);

        /**
         * @brief Returns the current configuration
         *
         * This method queries the robot the its configuration
         *
         * @return the current configuration
         */
        rw::math::Q getQ();

        /**
         * @brief Sends desired gripper position to the robot
         *
         * @param pos [in] the desired position of the gripper
         */
        void grip(float pos);

    private:
        crsA465::ACI _aci;
        bool _connected;
    };

    /** @} */
} // end namespaces

#endif // end include guard
