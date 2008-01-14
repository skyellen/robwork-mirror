#include "Task.hpp"

using namespace rw::task;

Task::Task(models::WorkCell* workcell) :
    _workcell(workcell)
{
    RW_ASSERT(_workcell);
}

Task::Task(std::auto_ptr<models::WorkCell> workcell) :
    _workcell(workcell.release()),
    _own_workcell(_workcell)
{
    RW_ASSERT(_workcell);
}

void Task::addTaskElement(const TaskElement& task_element)
{
	_task_elements.push_back(task_element);
}
