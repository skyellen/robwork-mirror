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

#ifndef RW_SENSOR_SCANNER25D_HPP
#define RW_SENSOR_SCANNER25D_HPP

#include "Scanner.hpp"
#include "Image25D.hpp"

namespace rw {
namespace sensor {

    /** @addtogroup sensor */
    /* @{ */

/**
 * @brief an interface describing a 3D scanner sensor. The scanner takes
 * pictures in the oposite direction of the z-axis of the frame that it is
 * attached to. The x-y plane forms the image plane such that the xy-origin is
 * located in the bottom left corner of the image.
 *
 */
class Scanner25D: public Scanner {

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner25D(rw::kinematics::Frame* frame, const std::string& name):
        Scanner(frame, name)
    {
    }

public:
    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner25D(){};

    /**
     * @brief gets the last acquired image
     * @return the image that was last acquired.
     */
    virtual const Image25D& getImage() = 0;


};
/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER3D_HPP*/
