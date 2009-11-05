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


#include "Task.hpp"

using namespace rw::task;
using namespace rw::models;
using namespace rw::common;

Task::Task(
    const Entity& entity,
    WorkCellPtr workcell,
    const std::vector<Action>& actions)
    :
    Entity(entity),
    _workcell(workcell),
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
