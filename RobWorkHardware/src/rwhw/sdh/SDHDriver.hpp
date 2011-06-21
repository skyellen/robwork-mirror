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

#ifndef RWHW_SDHDRIVER_HPP
#define RWHW_SDHDRIVER_HPP

/**
 * @file SDHDriver.hpp
 */



#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>
#include <rwhw/can/CanPort.hpp>
#include <boost/thread/thread.hpp>
#include <fstream>

namespace SDH {
	class cSDH;
}

namespace rwhw {

    /** @addtogroup sdh */
    /*@{*/

    /**
     * @brief Implements driver for the Schunk 3 finger hand. Only the controller can be
     * accessed through this class. If the tactile sensors need to be accessed then use
     * DSACON32 driver in rwha/tactile/DSACON32
     *
     * A position controller using a velocity ramp profile is used.
     */
    class SDHDriver {

    public:
        /**
         * @brief Creates driver object
         */
        SDHDriver();

        /**
         * @brief destructor
         */
        virtual ~SDHDriver();

        /**
         * @brief connects to the SDH hardware using the CAN interface. The SDH can be connected both through CAN and SerialPort
         * depending on the configuration of the SDH.
         */
        bool connect(int canNetId=0, int canBaudRate=1000000, double canTimeOut=0.5, int id_read=43, int id_write=42);

        /**
         * @brief connects to the SDH hardware using the CAN interface. The SDH can be connected both through CAN and SerialPort
         * depending on the configuration of the SDH.
         */
        bool connect( CanPort::Ptr cport, double canTimeOut=0.5, int id_read=43, int id_write=42);

        /**
         * @brief connects to the SDH hardware using the RS232 interface. The SDH can be connected both through CAN and SerialPort
         * depending on the configuration of the SDH.
         */
        bool connect(int port, unsigned long baudrate=115000, double timeout=0.5 );

        /**
         * @brief tests if the SDHDriver is connected to the hardware
         * @return true if it is connected
         */
        bool isConnected();

        /**
         * @brief disconnects from the hand
         */
        void disconnect();

        int getDOF();

        /**
		 * @brief sends a move to \b target command to the hand
		 * @param block [in] if true the method will block until the hand
		 * has reached the target or if it is not moving for a timeout period
		 */
        void moveCmd(bool block);

        /**
         * @brief sends a move to \b target command to the hand
         * @param target [in] the target to move to
         * @param block [in] if true the method will block until the hand
         * has reached the target or if it is not moving for a timeout period
         */
        void moveCmd(rw::math::Q target, bool block=false);


        /**
         * @brief Like moveCmd but this only moves one joint.
         * @param target
         * @param block
         */
        void moveJointCmd(int jointIdx, double target, bool block=false);

        /**
         * @brief waits until the joint positions has reached
         * the target of the last moveCmd or timeout occurs. A timeout of -1 means
         * wait indefinitely.
         * @param timeout [in] in seconds
         * @return true if target was reached, false otherwise
         */
        bool waitCmd(double timeout);

        /**
		 * @brief sets the wanted target in rad.
		 * @param jointPos
		 */
        void setTargetQ(const rw::math::Q& jointPos);

        /**
         * @brief sets the wanted target velocity. The velocity must be within
         * the velocity limits.
         * @param jointVel
         */
        void setTargetQVel(const rw::math::Q& jointVel);

        /**
         * @brief sets the wanted target Acceleration. The acceleration must be within
         * the acceleration limits.
         * @param jointAcc
         */
        void setTargetQAcc(const rw::math::Q& jointAcc);

        /**
         * @brief sets the wanted target Acceleration. The acceleration must be within
         * the acceleration limits.
         * @param jointAcc
         */
        void setTargetQCurrent(const rw::math::Q& jointCurr);

        /**
         * @brief queries the hand for its current target configuration
         * @return
         */
        rw::math::Q getTargetQ();

        /**
         * @brief queries the hand for its joint configuration.
         */
        rw::math::Q getQ();

        /**
         * @brief queries the hand for its current velocity.
         */
        rw::math::Q getdQ();

        /**
		 * @brief queries the hand for its current power use.
		 */
        rw::math::Q getQCurrent();

        /**
         * @brief stops the movement of all joints
         */
        void stop();

        /**
         * @brief stops the movement of all axes and switches of the
         * controllers.
         */
        void emergencyStop();

        /**
         * @brief
         */
        std::pair<rw::math::Q,rw::math::Q> getPosLimits();
        rw::math::Q getVelLimits();
        rw::math::Q getAccLimits();
        rw::math::Q getCurrentLimits();

        /**
         * @brief toggles if all joints are controlled or not
         * @param enabled [in] if true all joits are controlled, else all joints
         * are not controlled and can be freely manipulated by the user
         */
        void setJointEnabled(bool enabled);

        /**
         * @brief toggles if a joint is controlled or not
         * @param enabled [in] if true joint \b jointIdx is controlled, else it
         * is not controlled and can be freely manipulated by the user
         */
        void setJointEnabled(int jointIdx, bool enabled);

        /**
         * @brief sets the timeout of move commands
         * @param timeout
         */
        void setTimeout(double timeout);

        void updateLoop();
    protected:


        bool initConnection();
    private:

    	SDH::cSDH *_hand;
    	rw::math::Q _minPos,_maxPos;
    	std::vector<int> _axes;
    	std::vector<double> _vjointTargetVel, _vjointTarget;
    	std::vector<double> _vjointTmp;
    	double _moveCmdTimeout;
    };

    /**@}*/
} // end namespaces

#endif // end include guard
