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

#ifndef RWHW_FANUCDRIVER_HPP
#define RWHW_FANUCDRIVER_HPP

/**
 * @file FanucDriver.hpp
 */

#include "Fanuc.hpp"

namespace rwhw {

    /** @addtogroup rwhw */
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


        rw::math::Q getPos();

    private:
        size_t _updateRate;
        std::string _ipNr;


        // internal state variable for the initializing and closing the DLL
        bool _isLibraryOpen, _isConnected;
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

        bool _accCntChanged, _accFineChanged, _speedCntChanged,
            _speedFineChanged, _globalSpeedChanged;

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
        bool _resetError;
    };

    /**@}*/
} // end namespaces

#endif // end include guard
