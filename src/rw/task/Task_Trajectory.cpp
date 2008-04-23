#include "Trajectory.hpp"

#include <rw/kinematics/Kinematics.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::task;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::common;

Trajectory::Trajectory(
    const Entity& entity,
    WorkCell* workcell,
    Device* device,
    Frame* tool_frame)
    :
    Entity(entity),
    _workcell(workcell),
    _device(device),
    _tool_frame(tool_frame)
{
    RW_ASSERT(workcell);
    RW_ASSERT(device);
    RW_ASSERT(tool_frame);

	insert_link = true;
}

Trajectory::Trajectory(const Trajectory &trajectory) :
    Entity(trajectory)
{
    _workcell = trajectory._workcell;
    _device = trajectory._device;
    _tool_frame = trajectory._tool_frame;
	insert_link = true;

	Target* target;
	const Link* link = &*trajectory.link_list.begin();

	while(link != NULL) {
		addLink(*link);
		target = link->getNext();
		if(target != NULL) {
			addTarget(*target);
			link = target->getNext();
		}
		else
            link = NULL;
	}

	insert_link = trajectory.insert_link;
}

void Trajectory::addTarget(const Target &target)
{
	if (insert_link)
		addLink(
            Link(
                Entity("", PropertyMap()),
                NoConstraint()));

	target_list.push_back(target);
	Target* last_target = &target_list.back();

	Link* last_link = &link_list.back();
	last_link->setNext(last_target);
	last_target->setPrev(last_link);

	insert_link = true;
}

void Trajectory::addLink(const Link &link)
{
	if(insert_link == false)
		RW_THROW(
		"Error at link : "
		<< StringUtil::Quote(link.getName())
		<< "two links in a row. ");

	link_list.push_back(link);
	Link* last_link = &link_list.back();

	if(!target_list.empty()) {
		Target* last_target = &target_list.back();
		last_target->setNext(last_link);
		last_link->setPrev(last_target);
	}

	insert_link = false;
}
