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

#ifndef TACTILEARRAYUTIL_HPP_
#define TACTILEARRAYUTIL_HPP_

#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rw/sensor/TactileArray.hpp>

class TactileArrayUtil {

public:

    static std::vector<Contact3D>
        estimateContacts(const rw::sensor::TactileArray& arraySensor,
                         const rw::kinematics::State& state,
                         double minContactForce);

};


#endif /* TACTILEARRAYUTIL_HPP_ */
