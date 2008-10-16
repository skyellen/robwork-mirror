/*
 * SensorData.hpp
 *
 *  Created on: 31-08-2008
 *      Author: jimali
 */

#ifndef SENSORDATA_HPP_
#define SENSORDATA_HPP_

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


#endif /* SENSORDATA_HPP_ */
