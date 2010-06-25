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

#ifndef ACCELEGLOVE_HPP_
#define ACCELEGLOVE_HPP_

#include <rwhw/serialport/SerialPort.hpp>
namespace rwhw {

    struct AxisValue {
        int x,y,z;
    };

    struct AxisArray {
        AxisValue _values[6];
    };

    class AcceleGlove {

        virtual ~AcceleGlove(){};

        AcceleGlove* createInstance(rwhw::SerialPortPtr port);

        // the glove interface
        bool isConnected();

        bool connect();

        void close();

        int getSensorReadingForAxis(HandSensorPos sensorpos, Axis axis);

        AxisValue getSensorReadings(HandSensorPos sensorpos);

        AxisArray toArray();

        std::string toString();


    protected:
        AcceleGlove();
        rwhw::SerialPortPtr _port;

    };

}
#endif /* ACCELEGLOVE_HPP_ */
