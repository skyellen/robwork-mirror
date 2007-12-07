#include "TaskTrajectory.hpp"

using namespace rw::task;

TaskTrajectory::TaskTrajectory(std::string device, std::string tool_frame)
{
	insert_link = true;

	_device = device;
	_tool_frame = tool_frame;

}




void TaskTrajectory::addTarget(Target target)
{
	assert(insert_link == false);
	target_list.push_back(target);
	Target *last_target = &target_list.back();
	
	Link *last_link = &link_list.back();
	last_link->setNext(last_target);
	last_target->setPrev(last_link);

	insert_link = true;

}


void TaskTrajectory::addLink(Link link)
{
	assert(insert_link == true);
	link_list.push_back(link);
	Link *last_link = &link_list.back();

	if(!target_list.empty()) {
		Target *last_target = &target_list.back();
		last_target->setNext(last_link);
		last_link->setPrev(last_target);
	}

	insert_link = false;

}





