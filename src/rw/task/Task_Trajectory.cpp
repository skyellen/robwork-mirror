#include "Trajectory.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::task;

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

Trajectory::Trajectory(
    rw::models::WorkCell *workcell,
    rw::models::Device *device,
    rw::kinematics::Frame *tool_frame,
    const std::string& name)
    :
    _workcell(workcell),
    _device(device),
    _tool_frame(tool_frame),
    _name(name)
{
	insert_link = true;
}

Trajectory::Trajectory(const Trajectory &trajectory)
{
    _workcell = trajectory._workcell;
    _device = trajectory._device;
    _tool_frame = trajectory._tool_frame;
    _name = trajectory._name;

	insert_link = true;

	Target *target;
	const Link *link = &*trajectory.link_list.begin();

	while(link != NULL) {
		addLink(*link);
		target = link->next();
		if(target != NULL) {
			addTarget(*target);
			link = target->next();
		}
		else
		link = NULL;

	}

	insert_link = trajectory.insert_link;
}



void Trajectory::addTarget(const Target &target)
{
	if(insert_link == true)
		addLink(Link());


	target_list.push_back(target);
	Target *last_target = &target_list.back();
	
	Link *last_link = &link_list.back();
	last_link->setNext(last_target);
	last_target->setPrev(last_link);

	insert_link = true;
}

void Trajectory::addLink(const Link &link)
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


