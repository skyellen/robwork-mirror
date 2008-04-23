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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#include "Trajectory.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>

using namespace rw::task;
using namespace rw::kinematics;
using namespace rw::models;

Trajectory::Trajectory(
    const Entity& entity,
    WorkCell* workcell,
    Device* device,
    Frame* tool_frame,
    const std::vector<value_type>& elements)
    :
    Entity(entity),
    _workcell(workcell),
    _device(device),
    _tool_frame(tool_frame),
    _elements(elements.begin(), elements.end())
{
    RW_ASSERT(workcell);
    RW_ASSERT(device);
    RW_ASSERT(tool_frame);
}
