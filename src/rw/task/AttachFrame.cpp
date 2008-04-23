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

#include "AttachFrame.hpp"

#include <rw/common/macros.hpp>

using namespace rw::task;

AttachFrame::AttachFrame(
    const Entity& entity,
    rw::kinematics::MovableFrame *child,
    rw::kinematics::Frame *parent)
    :
    Entity(entity),
    _child(child),
    _parent(parent)
{
    RW_ASSERT(_child);
    RW_ASSERT(_parent);
}
