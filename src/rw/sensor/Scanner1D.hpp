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

#ifndef RW_SENSOR_SCANNER1D_HPP
#define RW_SENSOR_SCANNER1D_HPP

#include "Scanner.hpp"

namespace rw {
namespace sensor {

/**
 * @brief a one dimensional range scanner.
 */

class Scanner1D: public Scanner {
public:
    Scanner1D();
    virtual ~Scanner1D();

};

}
}

#endif /*RW_SENSOR_SCANNER3D_HPP*/
