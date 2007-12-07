/*********************************************************************
 * RobWork Version 0.2
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

#include "PlannerCollisionDetector.hpp"

#include <rw/common/macros.hpp>

#include <rw/proximity/CollisionDetector.hpp>
using namespace rw::proximity;

#include <rw/models/DeviceModel.hpp>
using namespace rw::models;

using namespace rw::pathplanning;
using namespace rw::kinematics;
using namespace rw::math;

PlannerCollisionDetector::PlannerCollisionDetector(
    CollisionDetector* detector,
    DeviceModel* device,
    const State& state)
    :
    _detector(detector),
    _device(device),
    _state(state)
{
    RW_ASSERT(_detector);
    RW_ASSERT(_device);
}

bool PlannerCollisionDetector::inCollision(const Q& q) const
{
    State state = _state;
    _device->setQ(q, state);
    return _detector->inCollision(state);
}
