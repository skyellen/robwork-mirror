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

#include "Proximity.hpp"
#include "ProximityCommon.hpp"

#include <rw/models/Accessor.hpp>
#include <rw/models/Models.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/ConveyorItem.hpp>
#include <rw/models/FixedJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PassivePrismaticFrame.hpp>
#include <rw/models/PassiveRevoluteFrame.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>
#include <map>

using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::models;

#define NS Proximity

//----------------------------------------------------------------------
// Static analysis of workcells

namespace
{
    typedef std::vector<Frame*> FrameList;
    typedef std::map<Frame*, FrameSet> FrameDependencyMap;

    bool inSet(Frame& frame, const FrameSet& set)
    {
        return set.count(&frame) != 0;
    }

    bool isDAF(const Frame& frame)
    {
        return Kinematics::isDAF(frame);
    }

    bool isFixedFrame(const Frame& frame)
    {
        return dynamic_cast<const FixedFrame*>(&frame) != 0;
    }

    bool isNonDAFFixedFrame(const Frame& frame)
    {
        return !isDAF(frame) && isFixedFrame(frame);
    }

    // The set of all frames of type FixedFrame that are not a DAF.
    FrameSet nonDAFFixedFrameSet(const WorkCell& workcell)
    {
        FrameSet result;
        BOOST_FOREACH(Frame* frame, Models::findAllFrames(workcell)) {
            if (isNonDAFFixedFrame(*frame)) result.insert(frame);
        }
        return result;
    }

    /**
       For each frame f a set of frames {f1, f2, ..., fN} is found.

       A pairing (f, fi) exists if and only if changes in the values for the
       frame of f can change the local transform (getTransform()) of fi.

       The frame f itself can be a member of {f1, f2, ..., fN}. This will for
       example be the case for Revolute and Prismatic joints.
    */
    FrameDependencyMap localTransformDependencies(
        const FrameList& workcellFrames)
    {
        FrameDependencyMap result;
        BOOST_FOREACH(Frame* f, workcellFrames) {
            if (dynamic_cast<MovableFrame*>(f)) {
                result[f].insert(f);
            }
            else if (dynamic_cast<FixedFrame*>(f)) {
            }
            else if (dynamic_cast<ConveyorItem*>(f)) {
                result[f].insert(f);
            }
            else if (PassivePrismaticFrame* pp =
                     dynamic_cast<PassivePrismaticFrame*>(f))
            {
                result[&pp->getOwner()].insert(f);
            }
            else if (PassiveRevoluteFrame* pp =
                     dynamic_cast<PassiveRevoluteFrame*>(f))
            {
                result[&pp->getOwner()].insert(f);
            }
            else if (dynamic_cast<FixedJoint*>(f)) {
                // Nothing to do: The local transform is fixed.
            }
            else if (dynamic_cast<PrismaticJoint*>(f)) {
                result[f].insert(f);
            }
            else if (dynamic_cast<RevoluteJoint*>(f)) {
                result[f].insert(f);
            }
            else {
                RW_THROW(
                    "Dependencies can not be computed for frame "
                    << *f
                    << " of unknown type.");
            }
        }
        return result;
    }
    FrameDependencyMap localTransformDependencies(
        const WorkCell& workcell)
    {
        return localTransformDependencies(Models::findAllFrames(workcell));
    }

    // Add to 'result' the subtree of 'frame' (including 'frame').
    void getSubTree(
        Frame& frame,
        const State& state,
        FrameSet& result)
    {
        result.insert(&frame);
        BOOST_FOREACH(Frame& child, frame.getChildren(state)) {
            getSubTree(child, state, result);
        }
    }

    /**
       For each frame f of 'controlled' a set of frames {f1, f2, ..., fN} is
       found.

       A pairing (f, fi) exists if and only if changes in the frame values of f
       produce changes in the global transform of fi.

       The function apply some simple rules of logic for how changes in joint
       values probagate. Thus for example the function can not numerically or
       analytically determine that the motions of a set of frames always cancel
       out (the function cannot deduce that the system has a closed loop).

       Typically f itself is a member of {f1, f2, ..., fN}.

       The computation clearly depends on the structure of the tree (the DAFs),
       and therefore a state is passed as argument.
    */
    FrameDependencyMap globalTransformDependencies(
        const FrameList& workcellFrames,
        const FrameList& controlled,
        const State& state)
    {
        const FrameDependencyMap localDependencies =
            localTransformDependencies(workcellFrames);

        FrameDependencyMap result;
        BOOST_FOREACH(Frame* root, controlled) {
            typedef FrameDependencyMap::const_iterator MI;
            const MI p = localDependencies.find(root);
            if (p != localDependencies.end()) {
                const FrameSet& dependent = p->second;
                BOOST_FOREACH(Frame* frame, dependent) {
                    getSubTree(*frame, state, result[root]);
                }
            }
        }
		return result;
    }
    FrameDependencyMap globalTransformDependencies(
        const WorkCell& workcell,
        const FrameList& controlled,
        const State& state)
    {
        const FrameList workcellFrames = Models::findAllFrames(workcell);
        return globalTransformDependencies(
            workcellFrames, controlled, state);
    }
    FrameDependencyMap globalTransformDependencies(
        const WorkCell& workcell,
        const State& state)
    {
        const FrameList workcellFrames = Models::findAllFrames(workcell);
        return globalTransformDependencies(
            workcellFrames, workcellFrames, state);
    }

    // The set of frames being *directly* (no passive joint search) modified by
    // changes in configuration of the device.
    FrameList getDeviceFrames(const Device& device)
    {
        const JointDevice* jd = dynamic_cast<const JointDevice*>(&device);
        if (!jd) RW_THROW("Device " << device << " is not of type JointDevice.");

        FrameList result;
        for (size_t i = 0; i < jd->getDOF(); i++) {
            result.push_back(jd->getActiveJoint(i));
        }
        return result;
    }

    // The union of the sets of frames being *directly* (no passive joint
    // search) modified by changes in configurations for the devices.
    FrameList getDeviceFrames(const std::vector<DevicePtr>& devices)
    {
        FrameList result;
        BOOST_FOREACH(const DevicePtr& ptr, devices) {
            const FrameList frames = getDeviceFrames(*ptr);
            result.insert(result.end(), frames.begin(), frames.end());
        }
        return result;
    }

    // The union of the set of frames dependent on \b controlled according to \b
    // dependencies.
    FrameSet dependencyUnion(
        const FrameDependencyMap& dependencies,
        const FrameList& controlled)
    {
        FrameSet result;
        BOOST_FOREACH(Frame* frame, controlled) {
            typedef FrameDependencyMap::const_iterator MI;
            const MI p = dependencies.find(frame);
            if (p != dependencies.end()) {
                Proximity::frameSetUnion(p->second, result);
            }
        }
        return result;
    }

    // The set of frames whose global transform is affected by changes in
    // configuration of \b devices for a state structure (DAFs are fixed) of \b
    // state.
    FrameSet globallyDependentFrames(
        const std::vector<DevicePtr>& devices,
        const FrameList& workcellFrames,
        const State& state)
    {
        const FrameList controlled = getDeviceFrames(devices);
        return
            dependencyUnion(
                globalTransformDependencies(workcellFrames, controlled, state),
                controlled);
    }

    /**
       The set of all frames that are can be considered *fixed* in local
       transform with respect to changes in the joint values of \b device. The
       \b device must be a joint device.

       The attachment of DAFs is irrelevant for this analysis.
    */
    FrameSet nonControlledFrameSet(
        const FrameList& workcellFrames,
        const Device& device)
    {
        // The set of frames of the device we control.
        const FrameList deviceFrames = getDeviceFrames(device);

        // Compute local dependencies for all frames in the workcell.
        const FrameDependencyMap localDependencies =
            localTransformDependencies(workcellFrames);

        // The implied set of all frames we control.
        FrameSet controlledFrameSet;
        BOOST_FOREACH(Frame* frame, deviceFrames) {
            typedef FrameDependencyMap::const_iterator DI;
            const DI p = localDependencies.find(frame);
            if (p != localDependencies.end()) {
                Proximity::frameSetUnion(p->second, controlledFrameSet);
            }
        }

        // The set of frames we don't control. These are the frames we consider
        // static.
        FrameSet result;
        BOOST_FOREACH(Frame* frame, workcellFrames) {
            if (controlledFrameSet.count(frame) == 0)
                result.insert(frame);
        }
        return result;
    }

    void staticFrameGroupsHelper(
        const State& state,
        const FrameSet& fixedFrames,
        Frame& root,
        FrameList& group,
        std::vector<FrameList>& result)
    {
        group.push_back(&root);

        BOOST_FOREACH(Frame& frame, root.getChildren(state)) {
            if (inSet(frame, fixedFrames)) {
                staticFrameGroupsHelper(state, fixedFrames, frame, group, result);
            }

            // Otherwise construct a new group for that entity.
            else {
                FrameList group;
                staticFrameGroupsHelper(state, fixedFrames, frame, group, result);
                if (group.size() > 1) result.push_back(group);
            }
        }
    }

    std::vector<FrameList> staticFrameGroups(
        const FrameSet& fixedFrames,
        Frame& worldFrame,
        const State& state)
    {
        std::vector<FrameList> result;
        FrameList group;
        staticFrameGroupsHelper(
            state,
            fixedFrames,
            worldFrame,
            group,
            result);
        if (group.size() > 1) result.push_back(group);
        return result;
    }

    std::vector<FrameList> staticFrameGroups(
        const FrameSet& fixedFrames,
        const WorkCell& workcell,
        const State& state)
    {
        return staticFrameGroups(fixedFrames, *workcell.getWorldFrame(), state);
    }

    bool frameShouldBeIncluded(
        const Frame& frame,
        CollisionStrategy* strategy,
        const CollisionSetup& setup)
    {
    	if(strategy!=NULL){
    		return strategy->hasModel(&frame) || setup.isVolatile(frame);
    	}

    	if (Accessor::collisionModelInfo().has(frame)) {
    		if (!Accessor::collisionModelInfo().get(frame).empty()) {
    			return true;
    		}
        }
        return setup.isVolatile(frame);
    }

    bool framePairShouldBeIncluded(
        const FramePair& pair,
        CollisionStrategy* strategy,
        const CollisionSetup& setup)
    {
        return
            frameShouldBeIncluded(*pair.first, strategy, setup) &&
            frameShouldBeIncluded(*pair.second, strategy, setup);
    }

    FramePair orderPair(const FramePair& pair)
    {
        if (pair.first->getName() < pair.second->getName())
            return pair;
        else
            return FramePair(pair.second, pair.first);
    }

    // All ordered pairs of frames (and no pairs (f, f)).
    FramePairList framePairList(const FrameList& frames)
    {
        FramePairList result;
        typedef FrameList::const_iterator I;
        for (I from = frames.begin(); from != frames.end(); ++from)
            for (I to = from + 1; to != frames.end(); ++to)
                result.push_back(orderPair(FramePair(*from, *to)));
        return result;
    }

    // All ordered pairs of frames (and no pairs (f, f)).
    FramePairSet framePairSet(const FrameList& frames)
    {
        FramePairSet result;
        typedef FrameList::const_iterator I;
        for (I from = frames.begin(); from != frames.end(); ++from)
            for (I to = from + 1; to != frames.end(); ++to)
                result.insert(orderPair(FramePair(*from, *to)));
        return result;
    }

    FramePairList filterFramePairList(
        const FramePairList& pairs,
        CollisionStrategy* strategy,
        const CollisionSetup& setup)
    {
        FramePairList result;
        BOOST_FOREACH(const FramePair& pair, pairs) {
            if (framePairShouldBeIncluded(pair, strategy, setup))
                result.push_back(pair);
        }
        return result;
    }

    FramePairList staticFramePairList(
        const FrameSet& fixedFrames,
        const WorkCell& workcell,
        const State& state)
    {
        FramePairList result;

        const std::vector<FrameList> groups =
            staticFrameGroups(fixedFrames, workcell, state);

        BOOST_FOREACH(const FrameList& group, groups) {
            const FramePairList pairs = framePairList(group);
            result.insert(result.end(), pairs.begin(), pairs.end());
        }

        return result;
    }

    FramePairSet staticFramePairSet(
        const FrameSet& fixedFrames,
        Frame& worldFrame,
        const State& state)
    {
        FramePairSet result;

        const std::vector<FrameList> groups =
            staticFrameGroups(fixedFrames, worldFrame, state);

        BOOST_FOREACH(const FrameList& group, groups) {
            const FramePairSet groupSet = framePairSet(group);
            result.insert(groupSet.begin(), groupSet.end());
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
    FramePairList filteredStaticFramePairList(
        const FrameSet& fixedFrames,
        const WorkCell& workcell,
        const State& state,
        CollisionStrategy* strategy,
        const CollisionSetup& setup)
    {
        if (setup.excludeStaticPairs()) {
            return filterFramePairList(
                staticFramePairList(fixedFrames, workcell, state),
                strategy,
                setup);
        } else
            return FramePairList();
    }

    std::string quote(const std::string& str) { return StringUtil::quote(str); }

    bool areInDifferentSets(
        const FramePair& pair,
        const FrameSet& u,
        const FrameSet& v)
    {
        Frame* a = pair.first;
        Frame* b = pair.second;
        return
            (inSet(*a, u) && inSet(*b, v)) ||
            (inSet(*b, u) && inSet(*a, v));
    }

    Frame& getWorldFrame(const Device& device, const State& state)
    {
        // getBase() returns a const pointer, so we const_cast.
        return const_cast<Frame&>(
            Kinematics::worldFrame(
                *device.getBase(),
                state));
    }

}

FramePairList NS::getExcludePairList(
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

FramePairSet NS::makeFramePairSet(
    const WorkCell& workcell,
    CollisionStrategy& strategy,
    const CollisionSetup& setup)
{
    FramePairSet result;

    // All pairs of frames to exclude.
    std::set<FramePair> exclude_set;

    // Pairs of frames that are statically linked and not DAFs.
    const FramePairList static_pairs =
        filteredStaticFramePairList(
            nonDAFFixedFrameSet(workcell),
            workcell,
            workcell.getDefaultState(),
            &strategy,
            setup);

    exclude_set.insert(static_pairs.begin(), static_pairs.end());

    // Pairs of frames specified in the exclude list.
    const FramePairList exclude_pairs = getExcludePairList(workcell, setup);
    exclude_set.insert(exclude_pairs.begin(), exclude_pairs.end());

    // All pairs of frames to consider.
    const FramePairList pairs =
        filterFramePairList(
            framePairList(Models::findAllFrames(workcell)),
            &strategy,
            setup);

    // Insert all pairs that are not excluded for some reason.
    BOOST_FOREACH(const FramePair& pair, pairs) {
        if (exclude_set.count(pair) == 0) {
            result.insert(pair);
        }
    }

    return result;
}

CollisionSetup NS::getCollisionSetup(const WorkCell& workcell)
{
    Frame& root = *workcell.getWorldFrame();
    if (Accessor::collisionSetup().has(root))
        return Accessor::collisionSetup().get(root);
    else
        return CollisionSetup();
}


FramePairSet NS::makeFramePairSet(
    const WorkCell& workcell,
    CollisionStrategy& strategy)
{
    return makeFramePairSet(workcell, strategy, getCollisionSetup(workcell));
}

FramePairSet NS::makeFramePairSet(
    const WorkCell& workcell)
{

    FramePairSet result;

    // All pairs of frames to exclude.
    std::set<FramePair> exclude_set;

    const CollisionSetup& setup = getCollisionSetup(workcell);

    // Pairs of frames that are statically linked and not DAFs.
    const FramePairList static_pairs =
        filteredStaticFramePairList(
            nonDAFFixedFrameSet(workcell),
            workcell,
            workcell.getDefaultState(),
            NULL,
            setup);

    exclude_set.insert(static_pairs.begin(), static_pairs.end());

    // Pairs of frames specified in the exclude list.
    const FramePairList exclude_pairs = getExcludePairList(workcell, setup);
    exclude_set.insert(exclude_pairs.begin(), exclude_pairs.end());

    // All pairs of frames to consider.
    const FramePairList pairs =
        filterFramePairList(
            framePairList(Models::findAllFrames(workcell)),
            NULL,
            setup);

    // Insert all pairs that are not excluded for some reason.
    BOOST_FOREACH(const FramePair& pair, pairs) {
        if (exclude_set.count(pair) == 0) {
            result.insert(pair);
        }
    }

    return result;
}


FramePairSet NS::makeFramePairSet(
    const Device& device,
    const State& state)
{
    // We take some effort here to avoid having to force the user to pass the
    // workcell as argument.

    // The world frame.
    Frame& worldFrame = getWorldFrame(device, state);

    // All frames of the workcell.
    const FrameList workcellFrames =
        Kinematics::findAllFrames(&worldFrame, state);

    // Pairs of frames that are statically linked.
    const FramePairSet excludeSet =
        staticFramePairSet(
            nonControlledFrameSet(workcellFrames, device),
            worldFrame,
            state);

    // All pairs of frames of the workcell.
    FramePairSet result = framePairSet(workcellFrames);

    // Compute 'result = result - excludeSet'.
    subtract(result, excludeSet);
    return result;
}

void NS::intersect(const FramePairSet& a, FramePairSet& b)
{
    std::vector<FramePair> erase;
    BOOST_FOREACH(const FramePair& pair, b) {
        if (a.count(pair) == 0)
            erase.push_back(pair);
    }
    BOOST_FOREACH(const FramePair& pair, erase) { b.erase(pair); }
}

void NS::subtract(FramePairSet& a, const FramePairSet& b)
{
    BOOST_FOREACH(const FramePair& pair, b) { a.erase(pair); }
}

void NS::frameSetUnion(const FrameSet& a, FrameSet& b)
{
    b.insert(a.begin(), a.end());
}

std::pair<FramePairSet, FramePairSet>
NS::makeStaticDynamicFramePairSet(
    const FramePairSet& workcellSet,
    const std::vector<rw::models::DevicePtr>& obstacleDevices,
    const std::vector<rw::models::DevicePtr>& controlledDevices,
    const rw::kinematics::State& state)
{
    // The world frame.
    Frame* worldFrame;
    if (!obstacleDevices.empty())
        worldFrame = &getWorldFrame(*obstacleDevices.front(), state);
    else if (!controlledDevices.empty()) {
        worldFrame = &getWorldFrame(*controlledDevices.front(), state);
    }
    else {
        RW_THROW("Obstacle devices or controlled devices expected.");
    }

    // All frames of the workcell.
    const FrameList workcellFrames = Kinematics::findAllFrames(worldFrame, state);

    // The set of frames being globally affected by \b obstacleDevices.
    const FrameSet obstacleDependent =
        globallyDependentFrames(obstacleDevices, workcellFrames, state);

    // The set of frames being globally affected by \b controlledDevices.
    const FrameSet deviceDependent =
        globallyDependentFrames(controlledDevices, workcellFrames, state);

    // The frame pairs for the static detector.
    FramePairSet staticSet = workcellSet;
    FramePairList staticRemove;
    BOOST_FOREACH(const FramePair& pair, staticSet) {
        Frame* a = pair.first;
        Frame* b = pair.second;
        if (inSet(*a, obstacleDependent) || inSet(*b, obstacleDependent))
            staticRemove.push_back(pair);
    }
    BOOST_FOREACH(const FramePair& pair, staticRemove) {
        staticSet.erase(pair);
    }

    // The frame pairs for the dynamic detector.
    FramePairSet dynamicSet;
    BOOST_FOREACH(const FramePair& pair, workcellSet) {
        if (areInDifferentSets(pair, obstacleDependent, deviceDependent))
            dynamicSet.insert(pair);
    }

    // Return the static and dynamic collision detector.
    return std::make_pair(staticSet, dynamicSet);
}
