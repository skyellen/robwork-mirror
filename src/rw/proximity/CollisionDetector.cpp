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

#include "CollisionDetector.hpp"
#include "CollisionStrategy.hpp"
#include "CollisionSetup.hpp"
#include "ProximityCommon.hpp"


// The FrameMap constructor.
#include <rw/models/Accessor.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <algorithm>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::geometry;

namespace
{
    bool isInList(const FramePair& pair,
    			  const FramePairList& pairs)
    {
        return std::find(pairs.begin(), pairs.end(), pair) != pairs.end();
    }

    bool pairCollides(CollisionStrategy& strategy,
    				  const FramePair& pair,
    				  const FKTable& fk)
    {
        const Frame* a = pair.first;
        const Frame* b = pair.second;
        return strategy.inCollision(a, fk.get(*a), b, fk.get(*b));
    }
    
}

CollisionDetector::CollisionDetector(WorkCell* workcell,
                                     CollisionStrategy* strategy):
    _firstContact(true),
    _root(workcell->getWorldFrame()),    
    _strategy(strategy),
    _state(workcell->getDefaultState())
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    try {
        _setup = Accessor::CollisionSetup().get(*workcell->getWorldFrame());
    } catch (const Exception& exp) {
    }
    initialize();
}

CollisionDetector::CollisionDetector(Frame* root,
                                     const CollisionSetup& setup,
                                     CollisionStrategy* strategy,
                                     const State& initialState):
    _firstContact(true),
    _root(root),
    _setup(setup),
    _strategy(strategy),
    _state(initialState)
{
    RW_ASSERT(root);
    RW_ASSERT(strategy);

    initialize();
}


void CollisionDetector::initialize()
{
    _collisionPairs.clear();
    
    // All frames reachable from the root.
    const std::vector<Frame*>& frames = Kinematics::FindAllFrames(_root, _state);

    // All pairs of frames.
    FramePairList pairs;
    typedef std::vector<Frame*>::const_iterator I;
    for (I from = frames.begin(); from != frames.end(); ++from) {
        if (_strategy->hasModel(*from)) {
            I to = from;
            for (++to; to != frames.end(); ++to) {
                if (_strategy->hasModel(*to))
                    pairs.push_back(FramePair(*from, *to));
            }
        }
    }

    // All pairs of frames to exclude.
    FramePairList exclude_pairs;
    const Kinematics::FrameMap& frameMap = Kinematics::BuildFrameMap(*_root, _state);
    const ProximityPairList& exclude = _setup.getExcludeList();
    
    typedef ProximityPairList::const_iterator EI;
    for (EI p = exclude.begin(); p != exclude.end(); ++p) {
        Frame* first = &ProximityCommon::lookupFrame(frameMap, p->first);
        Frame* second = &ProximityCommon::lookupFrame(frameMap, p->second);
        exclude_pairs.push_back(FramePair(first, second));
        exclude_pairs.push_back(FramePair(second, first));
    }

    
    // Include in the final list only the pairs that are not present in the
    // exclude list.
    typedef FramePairList::const_iterator PLI;
    for (PLI p = pairs.begin(); p != pairs.end(); ++p) {
    	if (std::find(exclude_pairs.begin(), exclude_pairs.end(), *p) == exclude_pairs.end()  ){
            _collisionPairs.push_back(*p);
    	} 
    }
}

CollisionDetector::~CollisionDetector()
{}

bool CollisionDetector::inCollision(const State& state,
                                    FramePairList* result) const
{
    FKTable fk(state);

    if (result) 
    	result->clear();

    bool found = false;
    typedef FramePairList::const_iterator I;

    for (I p = _collisionPairs.begin(); p != _collisionPairs.end(); ++p) {

        if (pairCollides(*_strategy, *p, fk)) {

            found = true;
            if (result) 
            	result->push_back(*p);
            if (_firstContact) 
            	break;
        }
    }

    return found;
}

void CollisionDetector::setCDStrategy(CollisionStrategy* strategy)
{
    RW_ASSERT(strategy);
    _strategy.reset(strategy);
}

bool CollisionDetector::addCollisionModel(const Frame* frame, const std::vector<Face<float> >& faces) {
    bool res = _strategy->addModel(frame, faces);
    if (res)
        initialize();
    return res;
}


void CollisionDetector::clearCache() {
    _strategy->clear();    
    initialize();
}
