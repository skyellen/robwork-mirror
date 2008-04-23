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

#include "Task.hpp"

using namespace rw::task;
using namespace rw::common;
using namespace rw::models;

Task::Task(
    const Entity& entity,
    rw::models::WorkCell* workcell,
    const std::vector<Action>& actions)
    :
    Entity(entity),
    _workcell(workcell),
    _actions(actions)
{
    RW_ASSERT(_workcell);
}

Task::Task(
    const Entity& entity,
    std::auto_ptr<WorkCell> workcell,
    const std::vector<Action>& actions)
    :
    Entity(entity),
    _workcell(workcell.release()),
    _own_workcell(_workcell),
    _actions(actions)
{
    RW_ASSERT(_workcell);
}

std::pair<Task::iterator, Task::iterator> Task::getActions()
{
    return std::make_pair(_actions.begin(), _actions.end());
}

std::pair<Task::const_iterator, Task::const_iterator> Task::getActions() const
{
    return std::make_pair(_actions.begin(), _actions.end());
}

std::ostream& rw::task::operator<<(std::ostream& out, const Task& task)
{
    return out
        << "Task["
        << task.getName()
        << ", "
        << task.getWorkCell()
        << "]";
}
