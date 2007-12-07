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

#ifndef rwlibs_devices_Fanuc_FanucVirtual_HPP
#define rwlibs_devices_Fanuc_FanucVirtual_HPP

/**
 * @file FanucVirtual.hpp
 */

#include "Fanuc.hpp"
#include "VelRampProfile.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Pose6D.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/State.hpp>

#include <string>

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
    /*@{*/

    /**
     * @brief Virtual Fanuc
     *
     * This class represents a Virtual Fanuc
     */

    class FanucVirtual: public Fanuc
    {
    public:
        /**
         * @brief Construct device
         * @param fanucModel [in] the DeviceModel to work on         
         * @param state [in] state of device
         */
        FanucVirtual(
            rw::models::SerialDevice* fanucModel,
            const rw::kinematics::State& state);

        /**
         * @brief Destructor
         */
        virtual ~FanucVirtual();

        /**
         * @copydoc Fanuc::setGlobalSpeed
         */
        virtual void setGlobalSpeed(int speed);

        /**
         * @copydoc Fanuc::getGlobalSpeed
         */
        virtual int getGlobalSpeed();

        /**
         * @copydoc Fanuc::moveFineQ
         */
        virtual void moveFineQ(rw::math::Q const &q);

        /**
         * @copydoc Fanuc::moveCntQ
         */
        virtual void moveCntQ(rw::math::Q const &q);

        /**
         * @copydoc Fanuc::isMoveComplete
         */
        virtual bool isMoveComplete();

        /**
         * @copydoc Fanuc::setCntQSpeed
         */
        virtual void setCntQSpeed(int speed);

        /**
         * @copydoc Fanuc::setFineQSpeed
         */
        virtual void setFineQSpeed(int speed);

        /**
         * @copydoc Fanuc::setCntQAcc
         */
        virtual void setCntQAcc(int acc);

        /**
         * @copydoc Fanuc::setFineQAcc
         */
        virtual void setFineQAcc(int acc);

        /**
         * @copydoc Fanuc::getQ
         */
        virtual rw::math::Q getQ();

        /**
         * @copydoc Fanuc::getQ
         */
        virtual rw::math::Q getdQ();

        /**
         * @copydoc Fanuc::update
         */
        virtual void update();

        /**
         * @copydoc Fanuc::connect
         */
        virtual bool connect();

        /**
         * @copydoc Fanuc::isConnected
         */
        virtual bool isConnected();

        /**
         * @copydoc Fanuc::disconnect
         */
        virtual void disconnect();

        /**
         * @copydoc Fanuc::callRobotProgram
         */
        virtual void callRobotProgram(unsigned int progNr);

        /**
         * @copydoc Fanuc::setCallArguments
         */
        virtual void setCallArguments(float arg1, float arg2, float arg3);

        /**
         * @copydoc Fanuc::isCallFinished
         */
        virtual bool isCallFinished();
                
        bool notifyProgram( ){ return true;};

        bool isCallWaiting( ){ return false;};

        float getLastError( );

    private:
        rw::models::SerialDevice *_model;
        rw::kinematics::State _state;

        // internal state variable for the initializing and closing the DLL
        bool _isConnected;

        // joint pos and velocity
        rw::math::Q _qCurrent, _dqCurrent;

        // timestamp for qCurrent and qdCurrent in ms
        long _timeStamp;
        //
        typedef enum{IdleCMD, CntCMD, FineCMD, CallCMD} CommandType;
        // The next command to execute
        CommandType _nextCmd,_lastCmd;
        bool _cmdChanged;
        //
        unsigned int _accCnt, _accFine, _speedCnt, _speedFine, _globalSpeed;
        bool _accCntChanged, _accFineChanged, _speedCntChanged,
            _speedFineChanged, _globalSpeedChanged;

        rw::math::Q _qValFine,_qValCnt,_qGoal;
        bool _qFineChanged,_qCntChanged;

        VelRampProfile *_velProfile;
        unsigned int _progNrToCall;
        boost::numeric::ublas::bounded_vector<float, 3> _callArg;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
