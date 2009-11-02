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


#include "SBLOptions.hpp"
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::models;

SBLOptions::SBLOptions(
    const PlannerConstraint& constraint,
    SBLExpandPtr expansion,
    QMetricPtr metric,
    double connectRadius)
    :
    constraint(constraint),
    expansion(expansion),
    metric(metric),
    connectRadius(connectRadius)
{
    resetCount = 20;
    // resetCount = 200;

    rootSampleInterval = 25;
    nodesPerCell = 10;

    nearNodeSelection = NearestNode;
    // nearNodeSelection = UniformSelect;
    // nearNodeSelection = NearestFromCell;

    // treeSelection = LargestTree;
    // treeSelection = SmallestTree;
    // treeSelection = WeightedTree;
    treeSelection = UniformTree;

    connectAt = ConnectAlways;
    // connectAt = ConnectAtReset;
}
