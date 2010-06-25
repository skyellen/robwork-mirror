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

#ifndef RWHW_DOCKWELDER_HPP
#define RWHW_DOCKWELDER_HPP




#include <rw/math/Q.hpp>

namespace rwhw {

    /** @addtogroup rwhw */
    /* @{ */

    /**
     * @brief Device for controlling the DockWelder robot.
     * The DockWelder robot is controlled through a ethernet socket connection.
     */
    class DockWelder {
    public:
        /**
         * @brief Constructor
         */
        DockWelder();

        /**
         * @brief Destructor
         */
        ~DockWelder();


        /**
         * @brief Opens the ethernet connection to the dockwelder controller.
         * @param serveraddr [in] address of the DockWelder controller
         */
        void openConnection(const std::string& serveraddr);


        void closeConnection();

        /**
         * @brief Turn on servos
         */
        void servoOn();

        /**
         * @brief Turn off servos
         */
        void servoOff();

        /**
         * @brief Set velocity....
         */
        void setVelocity(double velocity);

        /**
         * @brief Move to a specific configuration
         */
        void move(const rw::math::Q& q);

        /**
         * @brief Get the current configuration.
         */
        rw::math::Q getQ();

        /**
         *
         */
        void startMotion();

        void pauseMotion();

        void stopMotion();

        struct StatusStruct{
            time_t t;
            int isServoOn;
            int isLoaded;
            int isMoving;
            int isPaused;
            int isError;
            int isLimit;
            rw::math::Q q;
            int lj00; int hj00;
            int lj01; int hj01;
            int lj02; int hj02;
            int lj10; int hj10;
            int lj11; int hj11;
            int lj12; int hj12;
            int lj20; int hj20;
            int fj00;
            int fj01;
            int fj02;
            int fj10;
            int fj11;
            int fj12;
            int fj20;
            int nError; char errbuf[80];

            StatusStruct(): q(6) {};

        };

        typedef StatusStruct Status;

        Status status();
        void printStatus(std::ostream& ostr);




    private:
        void write(const char* buf);
        void read(char* buf);

    };

    /* @} */

} //end namespace rwhw

#endif //#ifndef RWHW_DOCKWELDER_HPP
