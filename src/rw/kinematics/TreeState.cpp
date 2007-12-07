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

#include "TreeState.hpp"
#include "Tree.hpp"
#include "Frame.hpp"

#include <rw/common/StringUtil.hpp>

using namespace rw::kinematics;
using namespace rw::common;

TreeState::TreeState():
    _tree(new Tree()) 
{}

TreeState::TreeState(boost::shared_ptr<Tree> tree) :
    _tree(tree)
{}

const Frame* TreeState::getParent(const Frame& frame) const
{
    return _tree->getDafParent(frame);
}

Frame* TreeState::getParent(Frame& frame) const
{
    return _tree->getDafParent(frame);
}

const std::vector<Frame*>& TreeState::getChildren(const Frame& frame) const
{
    return _tree->getChildren(frame);
}

void TreeState::attachFrame(Frame& frame, Frame& parent)
{
    // If it is not a DAF:
    Frame* static_parent = frame.getParent();
    if (static_parent)
        RW_THROW(
            "Can't attach frame "
            << StringUtil::Quote(frame.getName())
            << " to frame "
            << StringUtil::Quote(parent.getName())
            << ".\n"
            << "The frame is not a DAF but is statically attached to frame "
            << StringUtil::Quote(static_parent->getName()));

    if (&frame == &parent)
        RW_THROW(
            "Can't attach frame "
            << StringUtil::Quote(frame.getName())
            << " to itself.");

    // Take a copy of the tree. We can optimize here (easy todo) by taking a
    // copy only if the reference count is greater than one.
    _tree.reset(new Tree(*_tree));

    // Move the frame within this tree. Frame IDs are preserved.
    _tree->attachFrame(frame, parent);

    // Note that we are not updating the Q state. We don't have to because the
    // frame IDs have not changed in any way.
}
