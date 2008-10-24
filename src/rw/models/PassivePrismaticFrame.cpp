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

#include "PassivePrismaticFrame.hpp"
#include "PrismaticJoint.hpp"
#include <rw/math/EAA.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/macros.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

PassivePrismaticFrame::PassivePrismaticFrame(
    const std::string& name,
    const Transform3D<>& transform,
    Joint* owner,
    double scale,
    double offset)
    :
    Frame(0, name),
    _helper(PrismaticJoint::make(name, transform)),
    _owner(owner),
    _scale(scale),
    _offset(offset)
{
    RW_ASSERT(_owner);
}

void PassivePrismaticFrame::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    const double q_owner = *_owner->getQ(state);
    const double q = _scale * q_owner + _offset;

    _helper->getJointValueTransform(parent, q, result);
}

Transform3D<> PassivePrismaticFrame::getTransform(const State& state) const
{
    static const Transform3D<> id = Transform3D<>::identity();
    Transform3D<> result;
	Frame::getTransform(id, state, result);
    return result;
}
