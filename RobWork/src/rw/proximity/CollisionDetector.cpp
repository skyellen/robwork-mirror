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

#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;

CollisionDetector::CollisionDetector(CollisionStrategyPtr strategy,
                                     const FramePairSet& pairs) :
    _strategy(strategy), _collisionPairs(pairs)
{
    RW_ASSERT(strategy);
}

CollisionDetector::CollisionDetector(WorkCellPtr workcell,
                                     CollisionStrategyPtr strategy,
                                     const CollisionSetup& setup) :
    _strategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    _collisionPairs = Proximity::makeFramePairSet(*workcell, *strategy, setup);
}

CollisionDetector::CollisionDetector(WorkCellPtr workcell,
                                     CollisionStrategyPtr strategy) :
    _strategy(strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    _collisionPairs = Proximity::makeFramePairSet(*workcell, *strategy);
}




namespace
{
    bool pairCollides(
        CollisionStrategy& strategy,
        const FramePair& pair,
        const FKTable& fk)
    {
        const Frame* a = pair.first;
        const Frame* b = pair.second;
        return strategy.inCollision(a, fk.get(*a), b, fk.get(*b));
    }
}

bool CollisionDetector::inCollision(
    const State& state,
    FramePairSet* result,
    bool stopAtFirstContact) const
{
    FKTable fk(state);

    bool found = false;
    BOOST_FOREACH(const FramePair& pair, _collisionPairs) {
        if (pairCollides(*_strategy, pair, fk)) {
            found = true;
            if (result) {
                result->insert(pair);
                if (stopAtFirstContact) break;
            } else
                break;
        }
    }

    return found;
}

void CollisionDetector::setCollisionStrategy(CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    _strategy = strategy;
}



//----------------------------------------------------------------------
// Constructor functions

CollisionDetectorPtr CollisionDetector::make(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    return make(
        strategy,
        Proximity::makeFramePairSet(*workcell, *strategy));
}

CollisionDetectorPtr CollisionDetector::make(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    return make(
        strategy,
        Proximity::makeFramePairSet(*workcell, *strategy, setup));
}

CollisionDetectorPtr CollisionDetector::make(
    CollisionStrategyPtr strategy,
    const FramePairSet& pairs)
{
    RW_ASSERT(strategy);

    return ownedPtr(new CollisionDetector(strategy, pairs));
}

CollisionDetectorPtr CollisionDetector::make(
    const CollisionDetector& detector,
    const Device& device,
    const State& state)
{
    const FramePairSet workcellSet = detector.getFramePairSet();
    FramePairSet deviceSet = Proximity::makeFramePairSet(device, state);
    Proximity::intersect(workcellSet, deviceSet);

    return make(detector.getCollisionStrategyPtr(), deviceSet);
}

std::pair<CollisionDetectorPtr, CollisionDetectorPtr>
CollisionDetector::makeStaticDynamic(
    const CollisionDetector& detector,
    const std::vector<DevicePtr>& obstacleDevices,
    const std::vector<DevicePtr>& controlledDevices,
    const rw::kinematics::State& state)
{
    const std::pair<FramePairSet, FramePairSet> staticDynamic =
        Proximity::makeStaticDynamicFramePairSet(
            detector.getFramePairSet(),
            obstacleDevices,
            controlledDevices,
            state);
    CollisionStrategyPtr strategy = detector.getCollisionStrategyPtr();
    return std::make_pair(
        CollisionDetector::make(strategy, staticDynamic.first),
        CollisionDetector::make(strategy, staticDynamic.second));
}


bool CollisionDetector::addModel(
    const rw::kinematics::Frame* frame,
    const std::vector<rw::geometry::Face<float> >& faces)
{
	// add model to strategy
	_strategy->addModel(frame, faces);
	// and make sure the frame is all so used by collision detector

	// we run through the frame pair set, and for each frame found we combine it with frame
	_collisionPairs
}
