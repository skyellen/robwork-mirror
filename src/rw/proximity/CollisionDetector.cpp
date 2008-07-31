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

#include <rw/models/Accessor.hpp>
#include <rw/models/Models.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <boost/foreach.hpp>
#include <algorithm>
#include <set>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::geometry;

//----------------------------------------------------------------------
// Static analysis of workcells

namespace
{
    bool isDAF(const Frame& frame)
    {
        return frame.getParent() == 0;
    }

    bool isFixedFrame(const Frame& frame)
    {
        return dynamic_cast<const FixedFrame*>(&frame) != 0;
    }

    void staticFrameGroupsHelper(
        const State& state,
        Frame& root,
        std::vector<Frame*>& group,
        std::vector<std::vector<Frame*> >& result)
    {
        group.push_back(&root);

        BOOST_FOREACH(Frame& frame, root.getChildren(state)) {
            if (!isDAF(frame) && isFixedFrame(frame)) {
                staticFrameGroupsHelper(state, frame, group, result);
            }

            // Otherwise construct a new group for that entity.
            else {
                std::vector<Frame*> group;
                staticFrameGroupsHelper(state, frame, group, result);
                if (group.size() > 1) result.push_back(group);
            }
        }
    }

    std::vector<std::vector<Frame*> > staticFrameGroups(
        const WorkCell& workcell)
    {
        std::vector<std::vector<Frame*> > result;
        std::vector<Frame*> group;
        staticFrameGroupsHelper(
            workcell.getDefaultState(),
            *workcell.getWorldFrame(),
            group,
            result);
        if (group.size() > 1) result.push_back(group);
        return result;
    }

    bool frameShouldBeIncluded(
        const Frame& frame,
        CollisionStrategy& strategy,
        const CollisionSetup& setup)
    {
        return strategy.hasModel(&frame) || setup.isVolatile(frame);
    }

    FramePair orderPair(const FramePair& pair)
    {
        if (pair.first->getName() < pair.second->getName())
            return pair;
        else
            return FramePair(pair.second, pair.first);
    }

    FramePairList pairsOfFrames(
        const std::vector<Frame*>& frames,
        CollisionStrategy& strategy,
        const CollisionSetup& setup)
    {
        FramePairList result;
        typedef std::vector<Frame*>::const_iterator I;
        for (I from = frames.begin(); from != frames.end(); ++from) {
            Frame& a = **from;
            if (frameShouldBeIncluded(a, strategy, setup)) {
                for (I to = from + 1; to != frames.end(); ++to) {
                    Frame& b = **to;
                    if (frameShouldBeIncluded(b, strategy, setup)) {
                        result.push_back(orderPair(FramePair(&a, &b)));
                    }
                }
            }
        }
        return result;
    }

    /**
       @brief The list of frame pairs that because of their static
       relationships can be excluded from the collision setup.

       Only the frames that either have a CollisionModelID or are volatile
       are included.

       A non-empty list is returned only if setup.excludeStaticPairs() has been
       set to true.
    */
    FramePairList staticFramePairList(
        const WorkCell& workcell,
        CollisionStrategy& strategy,
        const CollisionSetup& setup)
    {
        FramePairList result;

        if (setup.excludeStaticPairs()) {
            std::vector<std::vector<Frame*> > groups = staticFrameGroups(workcell);
            BOOST_FOREACH(const std::vector<Frame*>& group, groups) {
                const FramePairList pairs = pairsOfFrames(group, strategy, setup);
                result.insert(result.end(), pairs.begin(), pairs.end());
            }
        }

        return result;
    }

    std::string quote(const std::string& str) { return StringUtil::quote(str); }

    FramePairList excludePairList(
        const WorkCell& workcell,
        const CollisionSetup& setup)
    {
        const ProximityPairList& exclude_pairs = setup.getExcludeList();

        FramePairList result;
        BOOST_FOREACH(const ProximityPair& pair, exclude_pairs) {
            Frame* a = workcell.findFrame(pair.first);
            if (!a) RW_WARN("No frame named " << quote(pair.first));

            Frame* b = workcell.findFrame(pair.second);
            if (!b) RW_WARN("No frame named " << quote(pair.second));

            if (a && b)
                result.push_back(orderPair(FramePair(a, b)));
        }
        return result;
    }
}

//----------------------------------------------------------------------

CollisionDetector::CollisionDetector(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup)
    :
    _firstContact(true),
    _strategy(strategy),
    _workcell(workcell),
    _setup(setup)
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    initialize(*_workcell, _setup);
}

CollisionDetector::CollisionDetector(
    WorkCellPtr workcell,
    CollisionStrategyPtr strategy)
    :
    _firstContact(true),
    _strategy(strategy),
    _workcell(workcell),
    _setup()
{
    RW_ASSERT(strategy);
    RW_ASSERT(workcell);

    Frame& root = *workcell->getWorldFrame();
    if (Accessor::collisionSetup().has(root))
        _setup = Accessor::collisionSetup().get(root);

    initialize(*_workcell, _setup);
}

void CollisionDetector::initialize(
    const WorkCell& workcell,
    const CollisionSetup& setup)
{
    _collisionPairs.clear();

    // All pairs of frames to exclude.
    std::set<FramePair> exclude_set;

    // Pairs of frames that are statically linked.
    const FramePairList static_pairs = staticFramePairList(workcell, *_strategy, setup);
    exclude_set.insert(static_pairs.begin(), static_pairs.end());

    // Pairs of frames specified in the exclude list.
    const FramePairList exclude_pairs = excludePairList(workcell, setup);
    exclude_set.insert(exclude_pairs.begin(), exclude_pairs.end());

    // All pairs of frames to consider.
    const FramePairList pairs =
        pairsOfFrames(Models::findAllFrames(workcell), *_strategy, setup);

    // Insert all pairs that are not excluded for some reason.
    BOOST_FOREACH(const FramePair& pair, pairs) {
        if (exclude_set.count(pair) == 0) {
            _collisionPairs.insert(pair);
        }
    }
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
    const State& state, FramePairList* result) const
{
    FKTable fk(state);

    if (result) result->clear();

    bool found = false;
    BOOST_FOREACH(const FramePair& pair, _collisionPairs) {
        if (pairCollides(*_strategy, pair, fk)) {
            found = true;
            if (result) result->push_back(pair);
            if (_firstContact) break;
        }
    }

    return found;
}

void CollisionDetector::setCollisionStrategy(CollisionStrategyPtr strategy)
{
    RW_ASSERT(strategy);
    _strategy = strategy;
}

bool CollisionDetector::addCollisionModel(
    const Frame* frame, const std::vector<Face<float> >& faces)
{
    const bool res = _strategy->addModel(frame, faces);
    if (res) initialize(*_workcell, _setup);
    return res;
}

void CollisionDetector::clearCache()
{
    _strategy->clear();
    initialize(*_workcell, _setup);
}

std::auto_ptr<CollisionDetector> CollisionDetector::make(
    rw::models::WorkCellPtr workcell,
    CollisionStrategyPtr strategy)
{
    typedef std::auto_ptr<CollisionDetector> T;
    return T(new CollisionDetector(workcell, strategy));
}

std::auto_ptr<CollisionDetector> CollisionDetector::make(
    rw::models::WorkCellPtr workcell,
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup)
{
    typedef std::auto_ptr<CollisionDetector> T;
    return T(new CollisionDetector(workcell, strategy, setup));
}
