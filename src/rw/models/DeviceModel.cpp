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

#include "DeviceModel.hpp"
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

Transform3D<double> DeviceModel::baseTframe(
    const Frame* f, const State& state) const
{
    return Kinematics::FrameTframe(getBase(), f, state);
}

Transform3D<double> DeviceModel::worldTbase(const State& state) const
{
    return Kinematics::WorldTframe(getBase(), state);
}

Transform3D<double> DeviceModel::baseTend(const State& state) const {
    return Kinematics::FrameTframe(getBase(), getEnd(), state);
}

std::ostream& rw::models::operator<<(std::ostream& out, const DeviceModel& device)
{
    return out << "Device[" << device.getName() << "]";
}
