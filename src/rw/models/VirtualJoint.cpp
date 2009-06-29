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

#include "VirtualJoint.hpp"

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

VirtualJoint::VirtualJoint(const std::string& name,
                           const Transform3D<>& transform,
                           size_t dof) :
    Joint(name, dof),
    _transform(transform)
{
}


void VirtualJoint::doMultiplyTransform(const Transform3D<>& parent,
                                     const State& state,
                                     Transform3D<>& result) const
{
    Transform3D<>::multiply(parent, _transform, result);
}


Transform3D<> VirtualJoint::doGetTransform(const State& state) const {
    return _transform;
}
