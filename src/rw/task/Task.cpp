#include "Task.hpp"

using namespace rw::task;

Task::Task(
    rw::models::WorkCell* workcell,
    const common::PropertyMap& properties,
    const std::string& name)
    :
    _workcell(workcell),
    _name(name),
    _properties(properties)
{
    RW_ASSERT(_workcell);
}

Task::Task(
    std::auto_ptr<rw::models::WorkCell> workcell,
    const common::PropertyMap& properties,
    const std::string& name)
    :
    _workcell(workcell.release()),
    _own_workcell(_workcell),
    _name(name),
    _properties(properties)
{
    RW_ASSERT(_workcell);
}

void Task::addTaskElement(const TaskElement& task_element)
{
	_task_elements.push_back(task_element);
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
