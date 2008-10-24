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

#ifndef RW_SENSOR_SCANNER2D_HPP
#define RW_SENSOR_SCANNER2D_HPP

#include "Scanner.hpp"

#include "Scan2D.hpp"

namespace rw {
namespace sensor {

/**
 * @brief The Scanner2D sensor encapsulate the basic interface of a
 * 2 dimensional range scanning device.
 */

class Scanner2D: public Scanner {

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner2D(rw::kinematics::Frame* frame, const std::string& name):
        Scanner(frame, name)
    {
    }

public:
    /**
     * @brief destructor
     */
    virtual ~Scanner2D();

    /**
     * @brief
     */
    virtual bool acquire() = 0;

    /**
     * @brief gets the last acquired scan
     *
     */
    virtual const Scan2D& getData() = 0;


    virtual std::pair<double,double> getRange();

    /**
     * @brief gets the scanning resolution in radians
     * @return
     */
    virtual double getResolution();

};

}
}

#endif /*RW_SENSOR_SCANNER2D_HPP_*/
