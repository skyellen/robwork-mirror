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

#include "DistanceCalculator.hpp"

#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/ProximityCommon.hpp>

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
    // Find the frame or crash.
    Frame& lookupFrame(const Kinematics::FrameMap& frameMap,
    				   const std::string& frameName)
    {
        const Kinematics::FrameMap::const_iterator pos = frameMap.find(frameName);
        if (pos == frameMap.end())
            RW_THROW("Frame "
                     << StringUtil::quote(frameName)
                     << " is not present in frame map.");

        return *pos->second;
    }

    bool isInList(const FramePair& pair,
    			  const FramePairList& pairs)
    {
        return std::find(pairs.begin(), pairs.end(), pair) != pairs.end();
    }
}

DistanceCalculator::DistanceCalculator(WorkCellPtr workcell,
                                       DistanceStrategyPtr strategy)
    :
    _shortestDistance(true),
    _root(workcell->getWorldFrame()),
    _strategy(strategy),
    _state(workcell->getDefaultState())

{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    try {
        _setup = Accessor::collisionSetup().get(*workcell->getWorldFrame());
    } catch (const Exception& exp) {
    }
    initialize();
}

DistanceCalculator::DistanceCalculator(Frame* root,
                                       const CollisionSetup& setup,
                                       DistanceStrategyPtr strategy,
                                       const State& initialState):
    _shortestDistance(true),
    _root(root),
    _setup(setup),
    _strategy(strategy),
    _state(initialState)
{
    RW_ASSERT(root);
    RW_ASSERT(strategy);

    initialize();
}

DistanceCalculator::DistanceCalculator(FramePairList pairs,
                                       DistanceStrategyPtr strategy):
    _distancePairs(pairs),
    _strategy(strategy)
{
    RW_ASSERT(strategy);
}

void DistanceCalculator::initialize()
{
    _distancePairs.clear();

    // All frames reachable from the root.
    const std::vector<Frame*>& frames = Kinematics::findAllFrames(_root, _state);

    // All pairs of frames.
    FramePairList pairs;
    typedef std::vector<Frame*>::const_iterator I;
    for (I from = frames.begin(); from != frames.end(); ++from) {
        if (_strategy->hasModel(*from)) {
            I to = from;
            for (++to; to != frames.end(); ++to) {
                if (_strategy->hasModel(*to)) {
                    pairs.push_back(FramePair(*from, *to));
                }
            }
        }
    }

    // All pairs of frames to exclude.
    FramePairList exclude_pairs;
    const Kinematics::FrameMap& frameMap = Kinematics::buildFrameMap(*_root, _state);
    const ProximityPairList& exclude = _setup.getExcludeList();

    typedef ProximityPairList::const_iterator EI;
    for (EI p = exclude.begin(); p != exclude.end(); ++p) {
        Frame* first = &lookupFrame(frameMap, p->first);
        Frame* second = &lookupFrame(frameMap, p->second);
        exclude_pairs.push_back(FramePair(first, second));
        exclude_pairs.push_back(FramePair(second, first));
    }

    // Include in the final list only the pairs that are not present in the
    // exclude list.
    typedef FramePairList::const_iterator PLI;
    for (PLI p = pairs.begin(); p != pairs.end(); ++p) {
        if (!isInList(*p, exclude_pairs))
            _distancePairs.push_back(*p);
    }
}

DistanceCalculator::~DistanceCalculator()
{
}

DistanceResult DistanceCalculator::distance(const State& state,
											std::vector<DistanceResult>* result) const
{
    FKTable fk(state);

    if (result != NULL)
    	result->clear();

    DistanceResult distance;
    distance.distance = DBL_MAX;
    typedef FramePairList::const_iterator I;
    for (I p = _distancePairs.begin(); p != _distancePairs.end(); ++p) {
        const Frame* a = p->first;
        const Frame* b = p->second;

        DistanceResult dist;
        bool res = _strategy->distance(dist, a, fk.get(*a), b, fk.get(*b));
        res = res; // To avoid a compiler warning.

        if (dist.distance < distance.distance)
            distance = dist;

        if (result != NULL)
            result->push_back(dist);
    }

    return distance;
}

DistanceResult DistanceCalculator::distance(const State& state,
                                            const Frame* frame,
                                            std::vector<DistanceResult>* result) const
{
    FKTable fk(state);

    if (result != NULL)
        result->clear();

    DistanceResult distance;
    distance.distance = DBL_MAX;
    typedef FramePairList::const_iterator I;
    for (I p = _distancePairs.begin(); p != _distancePairs.end(); ++p) {
        const Frame* a = p->first;
        const Frame* b = p->second;

        if (a == frame || b == frame) {

            DistanceResult dist;
            bool res = _strategy->distance(dist, a, fk.get(*a), b, fk.get(*b));
            res = res; // To avoid a compiler warning.

            if (dist.distance < distance.distance)
                distance = dist;

            if (result != NULL)
                result->push_back(dist);
        }
    }

    return distance;
}


void DistanceCalculator::setDistanceStrategy(DistanceStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    _strategy = strategy;
}

bool DistanceCalculator::addDistanceModel(const Frame* frame,
										  const std::vector<Face<float> >& faces)
{
    bool res = _strategy->addModel(frame, faces);
    if (res)
        initialize();
    return res;
}

void DistanceCalculator::clearCache()
{
    _strategy->clear();
    initialize();
}
