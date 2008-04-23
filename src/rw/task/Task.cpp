#include "Task.hpp"

using namespace rw::task;
using namespace rw::common;
using namespace rw::models;

Task::Task(
    const Entity& entity,
    rw::models::WorkCell* workcell)
    :
    Entity(entity),
    _workcell(workcell)
{
    RW_ASSERT(_workcell);
}

Task::Task(
    const Entity& entity,
    std::auto_ptr<WorkCell> workcell)
    :
    Entity(entity),
    _workcell(workcell.release()),
    _own_workcell(_workcell)
{
    RW_ASSERT(_workcell);
}

void Task::addAction(const Action& action)
{
	_actions.push_back(action);
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
