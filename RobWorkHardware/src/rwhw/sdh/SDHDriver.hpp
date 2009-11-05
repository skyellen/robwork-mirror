/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWHW_SDHDRIVER_HPP
#define RWHW_SDHDRIVER_HPP

/**
 * @file SDHDriver.hpp
 */



#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

#include <fstream>

namespace SDH {
	class cSDH;
}

namespace rwhw {

	class ESDCANPort;

    /** @addtogroup rwhw */
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

        int getDOF();

        /**
         * @brief connects to the hardware
         *
         * note: connect using RS232 or CAN
         */
        bool connect( ESDCANPort *cport/* can or rs232 */);

        /**
         * @brief tests if the SDHDriver is connected to the hardware
         * @return true if it is connected
         */
        bool isConnected();

        /**
         * @brief disconnects from the hand
         */
        void disconnect();

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
        void setTimeout(int timeout);
    private:

    	SDH::cSDH *_hand;
    	rw::math::Q _min,_max;
    	std::vector<int> _axes;
    	std::vector<double> _vjointTargetVel, _vjointTarget;
    	std::vector<double> _vjointTmp;
    };

    /**@}*/
} // end namespaces

#endif // end include guard
