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

#include "Proximity.hpp"

#include <rw/models/Accessor.hpp>
#include <rw/models/Models.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>

using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::models;

#define NS Proximity

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

    /**
       @brief The collision setup of the workcell.

       If no collision setup is stored in the workcell, then the empty
       collision setup is returned.

       @param workcell [in] Workcell containing a collision setup.
    */
    CollisionSetup getCollisionSetup(const WorkCell& workcell)
    {
        Frame& root = *workcell.getWorldFrame();
        if (Accessor::collisionSetup().has(root))
            return Accessor::collisionSetup().get(root);
        else
            return CollisionSetup();
    }
}

FramePairSet NS::makeFramePairSet(
    const WorkCell& workcell,
    CollisionStrategy& strategy,
    const CollisionSetup& setup)
{
    FramePairSet result;

    // All pairs of frames to exclude.
    std::set<FramePair> exclude_set;

    // Pairs of frames that are statically linked.
    const FramePairList static_pairs = staticFramePairList(workcell, strategy, setup);
    exclude_set.insert(static_pairs.begin(), static_pairs.end());

    // Pairs of frames specified in the exclude list.
    const FramePairList exclude_pairs = excludePairList(workcell, setup);
    exclude_set.insert(exclude_pairs.begin(), exclude_pairs.end());

    // All pairs of frames to consider.
    const FramePairList pairs =
        pairsOfFrames(Models::findAllFrames(workcell), strategy, setup);

    // Insert all pairs that are not excluded for some reason.
    BOOST_FOREACH(const FramePair& pair, pairs) {
        if (exclude_set.count(pair) == 0) {
            result.insert(pair);
        }
    }
    return result;
}

FramePairSet NS::makeFramePairSet(
    const WorkCell& workcell,
    CollisionStrategy& strategy)
{
    return makeFramePairSet(workcell, strategy, getCollisionSetup(workcell));
}
