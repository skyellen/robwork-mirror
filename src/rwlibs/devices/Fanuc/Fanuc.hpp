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

#ifndef rwlibs_devices_Fanuc_Fanuc_HPP
#define rwlibs_devices_Fanuc_Fanuc_HPP

/**
 * @file Fanuc.hpp
 */

#include <string>
#include <rw/math/Q.hpp>

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
    /*@{*/

    /**
     * @brief The interface for a fanuc robot.
     *
     * All internal values are updated using the update function. That means a
     * call to moveCntQ(q) is first sent to the robot when update() is called.
     */
    class Fanuc
    {
    protected:
        Fanuc() {}

    public:

        /**
         * @brief destruct object
         */
        virtual ~Fanuc() {}

        /**
         * @brief sets the speed in percentage of the maximum speed.
         *
         * The speed can be set in the interval [0;100]
         */
        virtual void setGlobalSpeed(int speed) = 0;

        /**
         * @brief gets the speed in percentage of the maximum speed.
         *
         * The speed is always in the interval [0;100]
         */
        virtual int getGlobalSpeed() = 0;

        /**
         * @brief sends a move FINE command to the fanuc controller.
         * The FINE command moves to the given Q and stops.
         *
         * @param q [in] The joint configuration to move to.
         */
        virtual void moveFineQ(rw::math::Q const &q) = 0;

        /**
         * @brief sends a move CNT command to the fanuc controller.
         * The CNT command moves toward the given Q and blends toward
         * the next move command if any is given before q is reached.
         *
         * @param q [in] The joint configuration to move to.
         */
        virtual void moveCntQ(rw::math::Q const &q) = 0;

        /**
         * @brief queries if the last move instuction is completed. Notice
         * that the CNT move instruction completes before reaching the
         * goal configuration.
         *
         * @return true if last move instruction is complete false otherwise.
         */
        virtual bool isMoveComplete() = 0;

        /**
         * @brief sets the speed of the moveCntQ instruction in percentage of
         * the globalspeed.
         *
         * @param speed [in] the speed in percent ]0;100]
         */
        virtual void setCntQSpeed(int speed) = 0;

        /**
         * @brief sets the speed of the moveFineQ instruction in percentage of
         * the globalspeed.
         *
         * @param speed [in] the speed in percent ]0;100]
         */
        virtual void setFineQSpeed(int speed) = 0;

        /**
         * @brief sets the acceleration override of the moveCntQ
         * instruction in percentage of "normal" acceleration were normal
         * is 100%.
         *
         * @param acc [in] the acceleration overide in percent ]0;500]
         */
        virtual void setCntQAcc(int acc) = 0;

        /**
         * @brief sets the acceleration override of the moveFineQ
         * instruction in percentage of "normal" acceleration were normal
         * is 100%.
         *
         * @param acc [in] the acceleration overide in percent ]0;500]
         */
        virtual void setFineQAcc(int acc) = 0;

        /**
         * @brief gets the joint configuration of the robot. The
         * configuration is only updated by calls to the update
         * function.
         *
         * @return the current joint configuration.
         */
        virtual rw::math::Q getQ() = 0;

        /**
         * @brief gets the joint velocities of the robot. The
         * configuration is only updated by calls to the update
         * function.
         *
         * @return the current joint velocity.
         */
        virtual rw::math::Q getdQ() = 0;

        /**
         * @brief updates all internal registers and transmits commands
         * to the robot controller.
         */
        virtual void update() = 0;

        /**
         * @brief connects to the robot controller
         * @return true if connect is succesfull, false otherwise
         */
        virtual bool connect() = 0;

        /**
         * @brief returns if connected to fanuc controller
         * @return true if connected, false otherwise
         */
        virtual bool isConnected() = 0;

        /**
         * @brief disconnects from the robot controller
         */
        virtual void disconnect() = 0;

        /**
         * @brief calls a program on the robot controller.
         */
        virtual void callRobotProgram(unsigned int progNr) = 0;

        /**
         * @brief We allow 3 arguments to be setup before calling a robot program.
         * @param arg1 [in] first argument
         * @param arg2 [in] second argument
         * @param arg3 [in] third argument
         */
        virtual void setCallArguments(float arg1, float arg2, float arg3) = 0;

        /**
         * @brief checks if a call to a robot program on the controller is 
         * finished
         */
        virtual bool isCallFinished() = 0;

        /**
         * @brief notifies the robot controller by setting the notify register
         * to 1.0
         * 
         * @return true if notify is posible, false otherwise. 
         * 
         * @note that the notification will happen at the next update
         */
        virtual bool notifyProgram( ) = 0;

        /**
         * @brief checks if the robot controller is waiting for a notify.
         * 
         * @return true if robot controller is waiting for notify false otherwise
         */
        virtual bool isCallWaiting( ) = 0;
        
        /**
         * @brief check last occored error
         * 
         * @return error code of last returned error
         */        
        virtual float getLastError( ) = 0;
        
        /**
         * @brief resets the error status
         */        
        virtual void resetError() = 0;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
