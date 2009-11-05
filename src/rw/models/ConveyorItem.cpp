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
    Transform3D<>::multiply(parent, getTransform(state), result);
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
