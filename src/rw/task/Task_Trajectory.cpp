#include "Trajectory.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::task;

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;


Trajectory::Trajectory(rw::models::WorkCell *workcell, rw::models::Device *device, rw::kinematics::Frame *tool_frame)
  : _workcell(workcell), _device(device), _tool_frame(tool_frame)
{
	insert_link = true;
}


void Trajectory::addTarget(const Target &target)
{
	assert(insert_link == false);
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

rw::math::Transform3D<> Trajectory::getBaseTransform(Target &target)
{
	rw::kinematics::State state = _workcell->getDefaultState();
	
	if(target.isToolLocation()) {
		Frame *base_frame = _device->getBase();
		Frame *target_frame = target.getToolLocation().getFrame();
		return Kinematics::FrameTframe(target_frame, base_frame, state);
	}
	else
	{
		Q q = target.getQ();
		_device->setQ(q,state);
		return _device->baseTend(state);

	}
}


rw::math::Transform3D<> Trajectory::getWorldTransform(Target &target)
{
	rw::kinematics::State state = _workcell->getDefaultState();

	if(target.isToolLocation()) {
		Frame *world_frame = _workcell->getWorldFrame();
		Frame *target_frame = target.getToolLocation().getFrame();
		return Kinematics::FrameTframe(target_frame, world_frame, state);	}
	else
	{
		Q q = target.getQ();
		_device->setQ(q,state);
		return _device->worldTbase(state) * _device->baseTend(state);

	}
}

/*
rw::interpolator::Pose6dStraightSegment Trajectory::getInterpolator(Link &link)
{
	assert(link.getMotionConstraint()==link.ToolConstraintE);

	ToolConstraint *tool_constraint = link.getToolContraint();
	return tool_constraint->Interpolator(getBaseTransform(*link.Prev()),getBaseTransform(*link.Next()));

}
*/
