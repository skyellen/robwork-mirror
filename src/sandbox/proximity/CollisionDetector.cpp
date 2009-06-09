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



