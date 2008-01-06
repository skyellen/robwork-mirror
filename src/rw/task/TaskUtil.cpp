#include "TaskUtil.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <iostream>
#include <string.h>



using namespace rw::task;

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

rw::math::Transform3D<> TaskUtil::getBaseTransform(const Trajectory &trajectory, const Target &target)
{
	rw::models::WorkCell *workcell = trajectory.getWorkCell();
	rw::models::Device *device = trajectory.getDevice();

	rw::kinematics::State state = workcell->getDefaultState();

	if(target.isToolLocation()) {
		Frame *base_frame = device->getBase();
		Frame *target_frame = target.getToolLocation().getFrame();
		assert(target_frame != NULL);
		return Kinematics::FrameTframe(target_frame, base_frame, state) * target.getToolLocation().getTransform();
	}
	else
	{
		Q q = target.getQ();
		device->setQ(q,state);
		return device->baseTend(state);

	}
}


rw::math::Transform3D<> TaskUtil::getWorldTransform(const Trajectory &trajectory, const Target &target)
{
	rw::models::WorkCell *workcell = trajectory.getWorkCell();
	rw::models::Device *device = trajectory.getDevice();
	rw::kinematics::State state = workcell->getDefaultState();

	return device->worldTbase(state) * getBaseTransform(trajectory,target);

}



rw::interpolator::Pose6dStraightSegment TaskUtil::getPoseInterpolator(const Trajectory &trajectory, const Link &link)
{
	assert(!link.isLinearJointConstraint());
	assert(!link.isNoConstraint());

	if(link.isCircularToolConstraint())
		return getCircularInterpolator(getBaseTransform(trajectory,*link.prev()),getBaseTransform(trajectory,*link.next()));
	else
		return getStraigtInterpolator(getBaseTransform(trajectory,*link.prev()),getBaseTransform(trajectory,*link.next()));

}


rw::interpolator::Pose6dStraightSegment TaskUtil::getStraigtInterpolator(rw::math::Transform3D<> a, rw::math::Transform3D<> b)
{
	return rw::interpolator::Pose6dStraightSegment(a, rw::math::VelocityScrew6D<>(b.P()-a.P(),inverse(a.R())*b.R()));
}



//Copy of linear interpolator - will be fixed later
rw::interpolator::Pose6dStraightSegment TaskUtil::getCircularInterpolator(rw::math::Transform3D<> a, rw::math::Transform3D<> b)
{
	return rw::interpolator::Pose6dStraightSegment(a, rw::math::VelocityScrew6D<>(b.P()-a.P(),inverse(a.R())*b.R()));

}

double TaskUtil::getLength(const Trajectory &trajectory, const rw::task::Link &link)
{
	Vector3D<> p_prev = getBaseTransform(trajectory,*link.prev()).P();
	Vector3D<> p_next = getBaseTransform(trajectory,*link.next()).P();

	return (p_next - p_prev).norm2();
}

