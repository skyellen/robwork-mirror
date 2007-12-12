#include "Task.hpp"

using namespace rw::task;


void Task::addTrajectory(Trajectory trajectory)
{
	_task_elements.push_back(trajectory);

}

void Task::addAction(Action action)
{
	_task_elements.push_back(action);

}

