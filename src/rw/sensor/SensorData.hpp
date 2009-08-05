/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_SENSOR_SENSORDATA_HPP
#define RW_SENSOR_SENSORDATA_HPP

namespace rw { namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief toplevel class for sensor data. Basicly describes interface for
 * setting and getting timestamps.
 */
class SensorData {
public:
	SensorData(long timeStamp=0):
	  _stamp(timeStamp)
	{}

    virtual long getTimeStamp(){
       return _stamp;
    }

    virtual void setTimeStamp(long timestamp){
        _stamp = timestamp;
    }

private:
	long _stamp;
};

/* @} */
}}

#endif /* RW_SENSOR_SENSORDATA_HPP */
