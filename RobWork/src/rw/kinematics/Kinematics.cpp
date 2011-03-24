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


#include "Kinematics.hpp"

#include "Frame.hpp"
#include "MovableFrame.hpp"
#include "FKRange.hpp"

#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>

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

Transform3D<> Kinematics::frameTframe(const Frame* from, const Frame* to,
                                      const State& state)
{
    RW_ASSERT(from != NULL);
    RW_ASSERT(to != NULL);
    FKRange range(from, to, state);
    return range.get(state);
}

//----------------------------------------------------------------------
// Kinematic tree traversals

namespace {
    void findAllFramesHelper(Frame& frame, const State& state,
                             std::vector<Frame*>& result)
    {
        result.push_back(&frame);
        BOOST_FOREACH(Frame& f, frame.getChildren(state)) {
            findAllFramesHelper(f, state, result);
        }
    }

    void findAllFramesHelper(Frame& frame, std::vector<Frame*>& result)
    {
        result.push_back(&frame);
        BOOST_FOREACH(Frame& f, frame.getChildren()) {
            findAllFramesHelper(f, result);
        }
    }

}

std::vector<Frame*> Kinematics::findAllFrames(Frame* root, const State& state)
{
    RW_ASSERT(root);
    std::vector<Frame*> result;
    findAllFramesHelper(*root, state, result);
    return result;
}

std::vector<Frame*> Kinematics::findAllFrames(Frame* root)
{
    RW_ASSERT(root);
    std::vector<Frame*> result;
    findAllFramesHelper(*root, result);
    return result;
}


Frame& Kinematics::worldFrame(Frame& frame, const State& state)
{
    Frame* parent = &frame;
    while (parent->getParent(state))
        parent = parent->getParent(state);
    return *parent;
}

const Frame& Kinematics::worldFrame(const Frame& frame, const State& state)
{
    // Forward to non-const version.
    return worldFrame(const_cast<Frame&> (frame), state);
}

std::vector<Frame*> Kinematics::childToParentChain(Frame* child, Frame* parent,
                                                   const State& state)
{
    typedef std::vector<Frame*> Vec;

    if (!child) {
        if (parent)
            RW_THROW("No parent chain from NULL to "
                    << StringUtil::quote(parent->getName()));

        return Vec();
    }

    Vec chain;
    for (Frame* frame = child; frame != parent; frame = frame->getParent(state)) {
        if (!frame) {
            const std::string parentName = parent ? StringUtil::quote(parent->getName()) : "NULL";

            RW_THROW("No parent chain from "
                    << StringUtil::quote(child->getName()) << " to "
                    << parentName);
        }

        chain.push_back(frame);
    }
    return chain;
}

std::vector<Frame*> Kinematics::reverseChildToParentChain(Frame* child,
                                                         Frame* parent,
                                                         const State& state)
{
    typedef std::vector<Frame*> V;
    const V chain = childToParentChain(child, parent, state);
    return V(chain.rbegin(), chain.rend());
}

std::vector<Frame*> Kinematics::parentToChildChain(Frame* parent, Frame* child,
                                                   const State& state)
{
    const std::vector<Frame*> chain = childToParentChain(child, parent, state);

    if (chain.empty())
        return chain;

    std::vector<Frame*> result;
    result.push_back(parent);
    result.insert(result.end(), chain.rbegin(), chain.rend() - 1);
    return result;
}

Kinematics::FrameMap Kinematics::buildFrameMap(Frame& root, const State& state)
{
    FrameMap result;
    BOOST_FOREACH(Frame* frame, Kinematics::findAllFrames(&root, state))
    {
        result.insert(std::make_pair(frame->getName(), frame));
    }
    return result;
}

//----------------------------------------------------------------------
// DAF manipulation

namespace {
    std::string quote(const std::string& str)
    {
        return StringUtil::quote(str);
    }

    Transform3D<> frameToFrame(const Frame& from, const Frame& to,
                               const State& state)
    {
        FKRange range(&from, &to, state);
        return range.get(state);
    }

    void attachFrame(State& state, Frame& frame, Frame& parent)
    {
        frame.attachTo(&parent, state);
    }

    void attachMovableFrame(State& state, MovableFrame& frame, Frame& parent,
                            const Transform3D<>& transform)
    {
        frame.setTransform(transform, state);
        attachFrame(state, frame, parent);
    }

    MovableFrame& getMovableFrame(Frame& frame)
    {
        MovableFrame* movable = dynamic_cast<MovableFrame*> (&frame);
        if (!movable)
            RW_THROW("Frame " << quote(frame.getName())
                    << " is not a movable frame.");
        return *movable;
    }

    void attachFrame(State& state,
                     Frame& frame,
                     Frame& parent,
                     const Transform3D<>& transform)
    {
        attachMovableFrame(state, getMovableFrame(frame), parent, transform);
    }
}

bool Kinematics::isDAF(const Frame& frame)
{
    // Unfortunately this reports the world frame to be a DAF!
    return (frame.getParent() == NULL);
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

void Kinematics::gripMovableFrame(State& state, MovableFrame& item,
                                  Frame& gripper)
{
    const Transform3D<>& relative = frameToFrame(gripper, item, state);
    attachMovableFrame(state, item, gripper, relative);
}

State Kinematics::grippedMovableFrame(const State& state, MovableFrame& item,
                                      Frame& gripper)
{
    State result = state;
    gripMovableFrame(result, item, gripper);
    return result;
}
