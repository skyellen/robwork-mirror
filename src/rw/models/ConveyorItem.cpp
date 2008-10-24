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

#include "ConveyorItem.hpp"

#include <rw/math/RPY.hpp>

using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;

ConveyorItem::ConveyorItem(const std::string& name) :
	Frame(7, name)
{}

ConveyorItem::~ConveyorItem() {}

Transform3D<> ConveyorItem::getTransform(const State& state) const
{
    const double* q = getQ(state);
    const RPY<> rpy(q[0], q[1], q[2]);
    const Vector3D<> pos(q[3], q[4], q[5]);
    return Transform3D<>(pos, rpy);
}

void ConveyorItem::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    Transform3D<>::transformMultiply(parent, getTransform(state), result);
}

void ConveyorItem::setTransformAndConveyorPosition(
    const Transform3D<>& transform,
    double conveyorPosition,
    State& state) const
{
    const RPY<> rpy(transform.R());
    const Vector3D<> pos(transform.P());

    double q[7];
    q[0] = rpy(0);
    q[1] = rpy(1);
    q[2] = rpy(2);
    q[3] = pos(0);
    q[4] = pos(1);
    q[5] = pos(2);
    q[6] = conveyorPosition;

    setQ(state, q);
}

double ConveyorItem::getConveyorPosition(const State& state) const
{
	const double* q = getQ(state);
	return q[6];
}
