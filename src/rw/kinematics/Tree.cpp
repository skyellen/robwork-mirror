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

#include "Tree.hpp"

#include "Frame.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
using namespace rw::common;

#include <vector>

using namespace rw::kinematics;

Tree::Tree() :
    _version(0)
{}

Tree::~Tree()
{}

void Tree::addFrame(Frame* frame)
{
    RW_ASSERT(!isInTree(*frame));

    const int id = allocateFrameID();
    frame->setID(id);

    // There is no turning back. Ownership is forever taken.
    _frames.at(id) = boost::shared_ptr<Frame>(frame);

    RW_ASSERT(isInTree(*frame));
}

void Tree::addFrameChain(Frame* first, Frame* last)
{
    FrameList frames;
    Frame* curr = last;

    while (curr && curr != first) {
        frames.push_back(curr);
        curr = curr->getParent();
    }

    if (!curr) {
         RW_THROW(
            "Frame chain from "
            << StringUtil::Quote(first->getName())
            << " to "
            << StringUtil::Quote(last->getName())
            << " is not connected/valid.");
    }

    RW_ASSERT(curr == first);
    frames.push_back(curr);

    typedef FrameList::const_iterator I;
    for (I p = frames.begin(); p != frames.end(); ++p) {
        addFrame(*p);
    }
}

void Tree::setDafParent(Frame& frame, Frame& parent)
{
    // They should both be in the tree.
    RW_ASSERT(isInTree(frame));
    RW_ASSERT(isInTree(parent));

    // The frame should not have a static parent and should not have a DAF
    // parent.
    RW_ASSERT(!frame.getParent());
    RW_ASSERT(!_parents.at(frame.getID()));

    _parents.at(frame.getID()) = &parent;
    addChild(parent, frame);
}

Frame* Tree::getDafParent(Frame& frame) const
{
    RW_ASSERT(isInTree(frame));
    return _parents.at(frame.getID());
}

const Frame* Tree::getDafParent(const Frame& frame) const
{
    // Forward to non-const version.
    return getDafParent(const_cast<Frame&>(frame));
}

const std::vector<Frame*>& Tree::getChildren(const Frame& frame) const
{
    RW_ASSERT(isInTree(frame));

    // We break const-correctness willfully here. If you have access to a tree
    // you can break soundness of the system easily anyway.
    return _children.at(frame.getID());
}

std::vector<Frame*> Tree::getFrames() const
{
    std::vector<Frame*> result;

    // We filter away the entries that are NULL.
    typedef std::vector<boost::shared_ptr<Frame> >::const_iterator I;
    for (I p = _frames.begin(); p != _frames.end(); ++p) {
        Frame* frame = (*p).get();
        if (frame)
            result.push_back(frame);
    }
    return result;
}

void Tree::addChild(const Frame& parent, Frame& frame)
{
    _children.at(parent.getID()).push_back(&frame);
}

void Tree::detachFrame(const Frame& frame)
{
    Frame& parent = *_parents.at(frame.getID());

    // Remove 'frame' from the child list of its parent.
    FrameList& children = _children.at(parent.getID());
    children.erase(
        std::remove(
            children.begin(),
            children.end(),
            &frame),
        children.end());

    // Remove the parent value for 'frame'.
    _parents.at(frame.getID()) = 0;
}

// True if the frame is registered in the tree.
bool Tree::isInTree(const Frame& frame) const
{
    const int id = frame.getID();
    return
        0 <= id && id < (int)_frames.size() &&
        _frames.at(id).get() == &frame;
}

int Tree::allocateFrameID()
{
    incVersion();

    if (_available_ids.empty()) {
        RW_ASSERT(
            _frames.size() == _children.size() &&
            _parents.size() == _children.size());

        const int id = _frames.size();

        _frames.push_back(boost::shared_ptr<Frame>());
        _parents.push_back(0);
        _children.push_back(FrameList());

        return id;
    } else {
        const int id = _available_ids.back();
        _available_ids.pop_back();
        return id;
    }
}

void Tree::attachFrame(Frame& frame, Frame& parent)
{
    RW_ASSERT(isInTree(frame));
    RW_ASSERT(isInTree(parent));

    if (frame.getParent())
        RW_THROW(
            "Frame "
            << StringUtil::Quote(frame.getName())
            << " is not a DAF.");

    // Detach the frame from its parent.
    detachFrame(frame);

    // Connect to the new parent.
    setDafParent(frame, parent);
}
