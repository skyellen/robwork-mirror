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


#include "Device.hpp"
#include <rw/math/Jacobian.hpp>
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

JacobianCalculatorPtr Device::baseJCend(const State& state) const
{
    return baseJCframe(getEnd(), state);
}

JacobianCalculatorPtr Device::baseJCframe(const Frame* frame,
                                                      const State& state) const
{
    std::vector<Frame*> frames(1, const_cast<Frame*>(frame)); // Dirty.
    return baseJCframes(frames, state);
}

// Streaming operator

std::ostream& rw::models::operator<<(std::ostream& out, const Device& device)
{
    return out << "Device[" << device.getName() << "]";
}
