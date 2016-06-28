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

#ifndef RW_SENSOR_SCANNER_HPP
#define RW_SENSOR_SCANNER_HPP

#include "Sensor.hpp"

namespace rw {
namespace sensor {

/**
 * @brief this interface describe a generic range scanning class.
 *
 */
class Scanner: public Sensor
{
protected:
    /**
     * @brief constructor
     * @param name [in] name of sensor
     * @param desc [in] description of sensor
     */
    Scanner(const std::string& name, const std::string& desc) :
        Sensor(name, desc)
    {
    }

    /**
     * @brief constructor
     * @param name [in] name of sensor
     */
    Scanner(const std::string& name) :
        Sensor(name)
    {
    }

public:

    virtual ~Scanner();

    /**
     * @brief Opens connection to the scanner
     */
    virtual void open() = 0;

    /**
     * @brief Returns whether the scanner has been opened
     *
     * @return true if scanner is opened
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Closes the connection to the scanner
     */
    virtual void close() = 0;

    /**
     * @brief Acquires data
     */
    virtual void acquire() = 0;

    /**
     * @brief tests whether an image has been acquired
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isScanReady() = 0;

    /**
     * @brief Returns the min and max range of this Scanner
     * @return min and max range
     */
    virtual std::pair<double, double> getRange() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;

};

}
}

#endif /*RW_SENSOR_SCANNER_HPP*/
