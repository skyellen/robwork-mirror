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

#include "RevoluteJoint.hpp"
#include "JointTransform.hpp"

#include <rw/math/EAA.hpp>
#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

RevoluteJoint::RevoluteJoint(
    Frame* parent,
    const std::string& name,
    const Transform3D<>& transform)
    :
    Joint(parent, name),
    _transform(transform)
{}

Transform3D<> RevoluteJoint::getTransform(const State& state) const
{
    const double q = *getQ(state);
    return JointTransform::getRevoluteTransform(_transform, q);
}
