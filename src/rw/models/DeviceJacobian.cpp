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

#include "DeviceJacobian.hpp"

#include <rw/kinematics/FKTable.hpp>
#include <rw/math/Jacobian.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

Jacobian DeviceJacobian::get(const State& state) const
{
    FKTable fk(state);
    return get(fk);
}

Jacobian DeviceJacobian::get(const FKTable& fk) const
{
    return doGet(fk);
}
