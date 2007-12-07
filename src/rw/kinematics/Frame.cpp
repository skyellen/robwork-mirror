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

#include "Frame.hpp"

#include "State.hpp"
#include "TreeState.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/Property.hpp>

#include <rw/kinematics/FKRange.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;

Frame::Frame(Frame* parent, int dof, const std::string& name) :
    _dof(dof),
    _id(-1),
    _name(name),
    _parent(parent)
{
    RW_ASSERT(0 <= dof);

    // It is OK for the parent to be NULL.
    if (parent)
        parent->addChild(this);
}

// Parents.

Frame* Frame::getParent(const State& state)
{
    Frame* f1 = getParent();
    if (f1 != NULL)
        return f1;
    else
        return getDafParent(state);
}

const Frame* Frame::getParent(const State& state) const
{
    const Frame* f1 = getParent();
    if (f1 != NULL)
        return f1;
    else
        return getDafParent(state);
}

const Frame* Frame::getDafParent(const State& state) const
{
    return state.getTreeState().getParent(*this);
}

Frame* Frame::getDafParent(const State& state)
{
    return state.getTreeState().getParent(*this);
}

// Children.

Frame::const_iterator_pair Frame::getChildren(const State& state) const
{
    return makeConstIteratorPair(
        _children,
        state.getTreeState().getChildren(*this));
}

Frame::iterator_pair Frame::getChildren(const State& state)
{
    return makeIteratorPair(
        _children,
        state.getTreeState().getChildren(*this));
}

Frame::const_iterator_pair Frame::getDafChildren(const State& state) const
{
    return makeConstIteratorPair(state.getTreeState().getChildren(*this));
}

Frame::iterator_pair Frame::getDafChildren(const State& state)
{
    return makeIteratorPair(state.getTreeState().getChildren(*this));
}

// Frame values.

const double* Frame::getQ(const State& state) const
{
    return state.getQState().getQ(*this);
}

void Frame::setQ(State& state, const double* vals) const
{
    state.getQState().setQ(*this, vals);
}

// Frame attachments.

void Frame::attachFrame(Frame& parent, State& state)
{
    state.getTreeState().attachFrame(*this, parent);
}

std::ostream& rw::kinematics::operator<<(std::ostream& out, const Frame& frame)
{
    return out << "Frame[" << frame.getName() << "]";
}
