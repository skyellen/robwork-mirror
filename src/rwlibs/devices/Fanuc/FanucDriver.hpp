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

#ifndef rwlibs_devices_Fanuc_FanucDriver_HPP
#define rwlibs_devices_Fanuc_FanucDriver_HPP

/**
 * @file FanucDriver.hpp
 */

#include "Fanuc.hpp"

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
    /*@{*/

    /**
     * @brief Only for 6 dof fanuc robots
     *
     */
    class FanucDriver : public Fanuc
    {
    public:
        /**
         * @brief construct object
         * @param ipNr [in] IP address of the robot controller
         * @param updateRate [in] the rate at which the controller recieves data.
         */
        FanucDriver(std::string ipNr, size_t updateRate);

        /**
         * @brief destruct object
         */
        virtual ~FanucDriver();

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
         * @copydoc Fanuc::disconnect
         */
        virtual void setCallArguments(float arg1, float arg2, float arg3);

        /**
         * @copydoc Fanuc::isCallFinished
         */
        virtual bool isCallFinished();

        bool notifyProgram( );

        bool isCallWaiting( );

        float getLastError(){return _error;};

        void resetError(){
                        
            _resetError = true; 
        };
                
    private:
        size_t _updateRate;
        std::string _ipNr;
        bool _resetError;
        rw::math::Q getPos();

        // internal state variable for the initializing and closing the DLL
        bool _isLibraryOpen,_isConnected;
        // joint pos and velocity
        rw::math::Q _qCurrent, _qdCurrent;
        // timestamp for qCurrent and qdCurrent in ms
        long _timeStamp;
        //
        typedef enum{IdleCMD, CntCMD, FineCMD, CallCMD} CommandType;
        // The next command to execute
        CommandType _nextCmd,_lastCmd;
        bool _cmdChanged;
        //
        unsigned int _accCnt, _accFine, _speedCnt, _speedFine, _globalSpeed;
        bool _accCntChanged, _accFineChanged, _speedCntChanged, _speedFineChanged, _globalSpeedChanged;

        rw::math::Q _qValFine,_qValCnt,_qValCntLast;
        bool _qFineChanged,_qCntChanged;

        // joint position handles for setting home, viaA and viaB configurations
        void *_qHomeReg,*_stateReg,*_qCntReg,*_qFineReg; //,*_qViaB;
        void *_movFineReg, *_movCntReg, *_accFineReg, *_accCntReg;

        void *_errorReg;

        // register handles for obtaining lock on via point A and B
        void *_notifyReg;//, *_lockB;

        void *_arg1Reg, *_arg2Reg, *_arg3Reg;

        void *_iHandle;
        unsigned int _progNrToCall;
        bool _isCallFinished,_notifyProgram,_isCallWaiting;
        boost::numeric::ublas::bounded_vector<float, 3> _callArg;

        float _error;
        int _updateCnt;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
