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

#ifndef RWHW_TRAKSTAR_HPP_
#define RWHW_TRAKSTAR_HPP_

//#include "TrakstarHelper.hpp"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <vector>

#include "RWATC3DG.hpp"

namespace rwhw {

    /**
     * @brief
     */
    class Trakstar {
    public:
        struct PoseData {
            rw::math::Vector3D<> pos;
            rw::math::Quaternion<> rot;
            double time;
            double quality; // quality from 0 to 1
            bool valid;
            USHORT status;
        };

    public:
        Trakstar(); // Constructor
        virtual ~Trakstar(); // Destructor

        void initialize(bool block=true);
        bool isInitialized(){ return _initStatus>=0;}

        int getInitStatus();

        std::vector<PoseData> getSensorValues();
        bool getSwitchValue();

        void startPolling();
        void stopPolling();

        int numberSensorsAttached();

        std::string getSensorStatusString(int errorcode);

    private:
        void initializeBird();
        void pollData(); // Should always run in a thread. Has (stoppable) while(1) loop.

    private:
        typedef DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON_RECORD TrakstarRawData;

        SYSTEM_CONFIGURATION      _ATC3DG;   // a pointer to a single instance of the system class
        std::vector<SENSOR_CONFIGURATION> _pSensor;  // a pointer to an array of sensor objects
        std::vector<TRANSMITTER_CONFIGURATION> _pXmtr;    // a pointer to an array of transmitter objects
        std::vector<TrakstarRawData> _rawValues;

        int             _errorCode;         // used to hold error code returned from procedure call
        int             _i;
        int             _sensorID;
        int             _transmitterID;
        short           _id;
        char            output[256];
        int             _numberBytes;

        int             _sensorsAttached;   // Is updated with the actual number of connected sensors at startPolling()

        boost::thread   _initThread;
        boost::thread   _pollThread;
        bool            _flagStopPoll;
        bool            _analogButtonOn;
        //DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD
        std::vector<PoseData> _record, _recordTmp;
        //int             _sensorStatus[NUMBER_OF_TRAKSTAR_SENSORS];


        boost::mutex    _mutexSensorValues;
        boost::mutex    _mutexInitBird;
        int             _initStatus;

        boost::condition_variable _conditionInitBird;


        bool _flagInitBird;

    };

}



#endif /* TRAKSTAR_HPP_ */
