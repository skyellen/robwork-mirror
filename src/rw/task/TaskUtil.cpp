#include "TaskUtil.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <iostream>
#include <string.h>

using namespace std;

using namespace rw::task;

using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::math;
using namespace rw::common;

rw::math::Transform3D<> TaskUtil::getBaseTransform(
    const Trajectory &trajectory, const Target &target)
{
	rw::models::WorkCell *workcell = &trajectory.getWorkCell();
	rw::models::Device *device = &trajectory.getDevice();

	rw::kinematics::State state = workcell->getDefaultState();

	if (target.isToolLocation()) {
		Frame *tcp_frame = &trajectory.getToolFrame();
		Transform3D<> end_to_tcp =
            Kinematics::FrameTframe(device->getEnd(),tcp_frame,state);

		Frame *base_frame = device->getBase();
		Frame *target_frame = target.getToolLocation().getFrame();

		return
            Kinematics::FrameTframe(base_frame,target_frame, state) *
            target.getToolLocation().getTransform()*end_to_tcp;
	}
	else
	{
		Q q = target.getQ();
		device->setQ(q,state);
		return device->baseTend(state);
	}
}

rw::math::Transform3D<> TaskUtil::getWorldTransform(
    const Trajectory &trajectory, const Target &target)
{
	rw::models::WorkCell& workcell = trajectory.getWorkCell();
	rw::models::Device& device = trajectory.getDevice();
	rw::kinematics::State state = workcell.getDefaultState();

	return device.worldTbase(state) * getBaseTransform(trajectory, target);
}

rw::interpolator::Pose6dStraightSegment TaskUtil::getPoseInterpolator(
    const Trajectory &trajectory, const Link &link)
{
	assert(!link.isLinearJointConstraint());
	assert(!link.isNoConstraint());

	if (link.isCircularToolConstraint())
		return getCircularInterpolator(
            getBaseTransform(trajectory, *link.prev()),
            getBaseTransform(trajectory,*link.next()));
	else
		return getStraigtInterpolator(
            getBaseTransform(trajectory,*link.prev()),
            getBaseTransform(trajectory,*link.next()));
}

rw::interpolator::Pose6dStraightSegment TaskUtil::getStraigtInterpolator(
    rw::math::Transform3D<> a, rw::math::Transform3D<> b)
{
	return rw::interpolator::Pose6dStraightSegment(
        a,
        rw::math::VelocityScrew6D<>(
            b.P()-a.P(), inverse(a.R())*b.R()));
}

//Copy of linear interpolator - will be fixed later
rw::interpolator::Pose6dStraightSegment TaskUtil::getCircularInterpolator(
    rw::math::Transform3D<> a, rw::math::Transform3D<> b)
{
	return rw::interpolator::Pose6dStraightSegment(
        a,
        rw::math::VelocityScrew6D<>(
            b.P()-a.P(),inverse(a.R())*b.R()));
}

double TaskUtil::getLength(
    const Trajectory &trajectory, const rw::task::Link &link)
{
	Vector3D<> p_prev = getBaseTransform(trajectory,*link.prev()).P();
	Vector3D<> p_next = getBaseTransform(trajectory,*link.next()).P();

	return (p_next - p_prev).norm2();
}

std::vector<rw::kinematics::State> TaskUtil::getStatePath(const Task &task)
{
	vector<State> statepath;

	State current_state;
	Device *device;

	Task::const_iterator it;
	for (it = task.begin(); it != task.end(); it++) {
		if(const Trajectory *trajectory = boost::get<Trajectory>(&*it)) {
			device = &trajectory->getDevice();
			current_state = trajectory->getState();
			for (Trajectory::const_link_iterator l_it = trajectory->link_begin();
                 l_it != trajectory->link_end();
                 l_it++)
            {
				rw::pathplanning::Path path = l_it->getSolvedPath();

				if (!path.empty()) {
					for (rw::pathplanning::Path::iterator p_it = path.begin();
                         p_it != path.end();
                         p_it++)
                    {
						device->setQ(*p_it,current_state);
						State state(current_state);
						statepath.push_back(state);
					}
				}
			}
		}

		if(const Action *action = boost::get<Action>(&*it)) {
			if (boost::get<AttachFrameAction>(&action->getActionType()))
            {
				State state(current_state);
				statepath.push_back(state);
			}
        }
	}

	return statepath;
}

Link TaskUtil::CombineLinks(const Link link1, const Link &link2)
{
	if(link1.isNoConstraint())
		return link2;
	if(link2.isNoConstraint())
		return link1;

    RW_THROW(
		"Error combinings links: "
		<< StringUtil::Quote(link1.getName()) << " and " << StringUtil::Quote(link2.getName()));
}
