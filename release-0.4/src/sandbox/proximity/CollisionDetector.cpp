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


#include "CollisionDetector.hpp"
#include "CollisionStrategy.hpp"
#include "CollisionSetup.hpp"
#include "Proximity.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/common/macros.hpp>

#include "StaticListFilter.hpp"


#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity::sandbox;

CollisionDetector::CollisionDetector(WorkCellPtr workcell,
                                     CollisionStrategyPtr strategy) :
    _strategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    _bpfilter = new StaticListFilter(workcell);
}

CollisionDetector::CollisionDetector(WorkCellPtr workcell,
                                     CollisionStrategyPtr strategy,
                                     BroadPhaseStrategyPtr bpfilter) :
    _strategy(strategy),
    _bpfilter(bpfilter)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);
}

bool CollisionDetector::inCollision(
    const State& state,
    CollisionResult* result,
    bool stopAtFirstContact) const
{
    _bpfilter->update(state);


    return false;
}

void CollisionDetector::setCollisionStrategy(CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    _strategy = strategy;
}



