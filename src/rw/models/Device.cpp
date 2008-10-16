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

#include "Device.hpp"
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

Transform3D<double> Device::baseTframe(
    const Frame* f, const State& state) const
{
    return Kinematics::frameTframe(getBase(), f, state);
}

Transform3D<double> Device::worldTbase(const State& state) const
{
    return Kinematics::worldTframe(getBase(), state);
}

Transform3D<double> Device::baseTend(const State& state) const {
    return Kinematics::frameTframe(getBase(), getEnd(), state);
}

// Jacobians

Jacobian Device::baseJend(const State& state) const
{
    return baseJframe(getEnd(), state);
}

Jacobian Device::baseJframe(const Frame* frame,
                            const State& state) const
{
    std::vector<Frame*> frames(1, const_cast<Frame*>(frame)); // Dirty.
    return baseJframes(frames, state);
}

boost::shared_ptr<DeviceJacobian> Device::baseDJend(const State& state) const
{
    return baseDJframe(getEnd(), state);
}

boost::shared_ptr<DeviceJacobian> Device::baseDJframe(const Frame* frame,
                                                      const State& state) const
{
    std::vector<Frame*> frames(1, const_cast<Frame*>(frame)); // Dirty.
    return baseDJframes(frames, state);
}

// Streaming operator

std::ostream& rw::models::operator<<(std::ostream& out, const Device& device)
{
    return out << "Device[" << device.getName() << "]";
}
