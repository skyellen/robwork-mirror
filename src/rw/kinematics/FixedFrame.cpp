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

#include "FixedFrame.hpp"

using namespace rw::kinematics;
using namespace rw::math;

FixedFrame::FixedFrame(
    const std::string& name,
    const Transform3D<>& transform)
    :
    Frame(0, name),
    _transform(transform)
{}

Transform3D<> FixedFrame::getTransform(const State& state) const
{
    return _transform;
}

void FixedFrame::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    Transform3D<>::transformMultiply(parent, _transform, result);
}
