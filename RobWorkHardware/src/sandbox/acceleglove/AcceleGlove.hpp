/*
 * AcceleGlove.hpp
 *
 *  Created on: 13-11-2009
 *      Author: jimali
 */

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
