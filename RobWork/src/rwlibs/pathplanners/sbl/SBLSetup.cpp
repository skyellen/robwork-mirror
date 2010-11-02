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


#include "SBLSetup.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;

SBLSetup SBLSetup::make(
    const PlannerConstraint& constraint,
    SBLExpandPtr expansion,
	QMetric::Ptr metric,
    double connectRadius)
{
    return SBLSetup(SBLOptions(constraint, expansion, metric, connectRadius));
}

SBLSetup SBLSetup::make(
    const PlannerConstraint& constraint,
	Device::Ptr device,
    double expandRadius,
    double connectRadius)
{
    if (expandRadius < 0) expandRadius = 0.25;
    if (connectRadius < 0) connectRadius = 0.5;

    return make(
        constraint,
        SBLExpand::makeShrinkingUniformBox(
            constraint.getQConstraintPtr(),
            device->getBounds(),
            2 * expandRadius),
        PlannerUtil::normalizingInfinityMetric(
            device->getBounds()),
        connectRadius);
}
