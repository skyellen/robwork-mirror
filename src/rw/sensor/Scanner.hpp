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

#ifndef RW_SENSOR_SCANNER_HPP
#define RW_SENSOR_SCANNER_HPP

#include "Sensor.hpp"
#include <rw/kinematics/State.hpp>

namespace rw {
namespace sensor {

class Scanner: public Sensor {
protected:

    Scanner(rw::kinematics::Frame* frame, const std::string& name):
        Sensor(frame,name)
    {
    }


public:
    /**
     * @brief Opens connection to the scanner
     */
    virtual void open() = 0;

    /**
     * @brief Returns whether the scanner has been opened
     *
     * @return true if camera is opened
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Closes the connection to the scanner
     */
    virtual void close() = 0;

    /**
     * @brief Acquires data
     */
    virtual void acquire( const rw::kinematics::State& state ) = 0;

    /**
     * @brief tests whether an image has been acquired
     * @return true if an image has been acquired, false otherwise.
     */
    virtual bool isImageReady() = 0;

    /**
     * @brief Returns the min and max range of this Scanner3D
     * @return min and max range
     */
    virtual std::pair<double,double> getRange() = 0;

    /**
     * @brief returns the framerate that this camera is setup with
     * @return the framerate in frames per second
     */
    virtual double getFrameRate() = 0;

};

}
}

#endif /*RW_SENSOR_SCANNER_HPP*/
