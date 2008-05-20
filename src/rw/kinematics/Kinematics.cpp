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

#include "Kinematics.hpp"

#include "MovableFrame.hpp"
#include "FKRange.hpp"

#include <rw/common/StringUtil.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;

//----------------------------------------------------------------------
// Kinematics computation

Transform3D<> Kinematics::worldTframe(const Frame* to, const State& state)
{
    const Frame* f = to;

    Transform3D<> transform = Transform3D<>::identity();
    while (f) {
        transform = f->getTransform(state) * transform;
        f = f->getParent(state);
    }

    return transform;
}

Transform3D<> Kinematics::frameTframe(
    const Frame* from,
    const Frame* to,
    const State& state)
{
    RW_ASSERT(from != NULL);
    RW_ASSERT(to != NULL);
    FKRange range(from, to, state);
    return range.get(state);
}

//----------------------------------------------------------------------
// Kinematic tree traversals

namespace
{
    void findAllFramesHelper(
        Frame& frame,
        const State& state,
        std::vector<Frame*>& result)
    {
        //std::cout << "\nId:"<< frame.getID();
        result.push_back(&frame);
        Frame::iterator_pair children = frame.getChildren(state);
        //for (Frame::iterator it = children.first; it != children.second; ++it) {
        //    std::cout << " c:" << (*it).getID();
        //}     
        children = frame.getChildren(state);
        for (Frame::iterator it = children.first; it != children.second; ++it) {
            findAllFramesHelper(*it, state, result);
        }
    }
}

std::vector<Frame*> Kinematics::findAllFrames(
    Frame* root, const State& state)
{
    RW_ASSERT(root);
    //std::cout << "4";
    std::vector<Frame*> result;
    //std::cout << "4";
    findAllFramesHelper(*root, state, result);
    //std::cout << "4";
    return result;
}

std::vector<Frame*> Kinematics::childToParentChain(
    Frame* child, Frame* parent, const State& state)
{
    typedef std::vector<Frame*> Vec;

    if (!child) {
        if (parent)
            RW_THROW(
                "No parent chain from NULL to "
                << StringUtil::quote(parent->getName()));

        return Vec();
    }

    Vec chain;
    for (Frame* frame = child;
         frame != parent;
         frame = frame->getParent(state))
    {
        if (!frame) {
            const std::string parentName =
                parent ?
                StringUtil::quote(parent->getName()) :
                "NULL";

            RW_THROW(
                "No parent chain from "
                << StringUtil::quote(child->getName())
                << " to "
                << parentName);
        }

        chain.push_back(frame);
    }
    return chain;
}

std::vector<Frame*> Kinematics::reverseChildToParentChain(
    Frame* child, Frame* parent, const State& state)
{
    typedef std::vector<Frame*> V;
    const V chain = childToParentChain(child, parent, state);
    return V(chain.rbegin(), chain.rend());
}

std::vector<Frame*> Kinematics::parentToChildChain(
    Frame* parent, Frame* child, const State& state)
{
    const std::vector<Frame*> chain = childToParentChain(child, parent, state);

    if (chain.empty()) return chain;

    std::vector<Frame*> result;
    result.push_back(parent);
    result.insert(result.end(), chain.rbegin(), chain.rend() - 1);
    return result;
}

Kinematics::FrameMap Kinematics::buildFrameMap(
    Frame& root, const State& state)
{
    FrameMap result;

    const std::vector<Frame*>& frames = Kinematics::findAllFrames(&root, state);
    typedef std::vector<Frame*>::const_iterator I;
    for (I p = frames.begin(); p != frames.end(); ++p) {
        result.insert(std::make_pair((**p).getName(), *p));
    }

    return result;
}

//----------------------------------------------------------------------
// DAF manipulation

namespace
{
    std::string quote(const std::string& str) { return StringUtil::quote(str); }

    Transform3D<> frameToFrame(
        const Frame& from,
        const Frame& to,
        const State& state)
    {
        FKRange range(&from, &to, state);
        return range.get(state);
    }

    void attachFrame(State& state, Frame& frame, Frame& parent)
    {
        frame.attachTo(&parent, state);
    }

    void attachMovableFrame(
        State& state,
        MovableFrame& frame,
        Frame& parent,
        const Transform3D<>& transform)
    {
        frame.setTransform(transform, state);
        attachFrame(state, frame, parent);
    }

    MovableFrame& getMovableFrame(Frame& frame)
    {
        MovableFrame* movable = dynamic_cast<MovableFrame*>(&frame);
        if (!movable)
            RW_THROW(
                "Frame "
                << quote(frame.getName())
                << " is not a movable frame.");
        return *movable;
    }

    void attachFrame(
        State& state,
        Frame& frame,
        Frame& parent,
        const Transform3D<>& transform)
    {
        attachMovableFrame(state, getMovableFrame(frame), parent, transform);
    }
}

void Kinematics::gripFrame(State& state, Frame& item, Frame& gripper)
{
    const Transform3D<>& relative = frameToFrame(gripper, item, state);
    attachFrame(state, item, gripper, relative);
}

State Kinematics::grippedFrame(const State& state, Frame& item, Frame& gripper)
{
    State result = state;
    gripFrame(result, item, gripper);
    return result;
}

void Kinematics::gripMovableFrame(
    State& state,
    MovableFrame& item,
    Frame& gripper)
{
    const Transform3D<>& relative = frameToFrame(gripper, item, state);
    attachMovableFrame(state, item, gripper, relative);
}

State Kinematics::grippedMovableFrame(
    const State& state,
    MovableFrame& item,
    Frame& gripper)
{
    State result = state;
    gripMovableFrame(result, item, gripper);
    return result;
}
