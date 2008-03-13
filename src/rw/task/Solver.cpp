#include "Solver.hpp"

using namespace rw::task;
using namespace std;

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;

#include "TaskUtil.hpp"

#include <rw/kinematics/Kinematics.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

rw::invkin::IKMetaSolver *make_meta_solver(
    rw::models::Device *device)
{
	return new rw::invkin::IKMetaSolver(
        new rw::invkin::ResolvedRateSolver(device),
        device,
        NULL);
}

class LinkVisitor : public boost::static_visitor<Path>
{
public:
	LinkVisitor(
        Trajectory *trajectory,
        const Link &link,
        State *current_state)
		:
        _trajectory(trajectory),
        _link(link),
        _current_state(current_state)
	{
		_current_q = _trajectory->getDevice().getQ(*current_state);
	}

	static void setPlanners(
        rw::pathplanning::PathPlanner *path_planner,
        TrajectoryPlanner *trajectory_planner,
        IKMetaSolver *meta_solver)
	{
		_meta_solver = meta_solver;
		_path_planner = path_planner;
		_trajectory_planner = trajectory_planner;
	}

    Path operator()(NoConstraint i)
    {
		Path path;

		Q q_end = Solver::calcQ(*_trajectory,*_link.next(),*_current_state);

		if (!_path_planner->query(_current_q,q_end,path,60))
			RW_THROW(
			"Error in motionplanner solving along link: "
			<< StringUtil::Quote(_link.getName()));

		return path;
    }

    Path operator()(LinearJointConstraint i)
    {
		Path path;
		path.push_back(_current_q);

		Q q_end = Solver::calcQ(*_trajectory,*_link.next(),*_current_state);

		path.push_back(q_end);

		return path;
    }

	Path operator()(LinearToolConstraint i)
    {
		Path path;
		if (!_trajectory_planner->solve(
                _current_q,
                TaskUtil::getPoseInterpolator(
                    *_trajectory,
                    _link),
                path))
			RW_THROW(
                "Error when solving CircularToolConstraint along link: "
                << StringUtil::Quote(_link.getName()));

		return path;
    }

	Path operator()(CircularToolConstraint i)
    {
		Path path;
		if (!_trajectory_planner->solve(
               _current_q,
               TaskUtil::getPoseInterpolator(
                   *_trajectory,
                   _link),
               path))
			RW_THROW(
                "Error when solving CircularToolConstraint along link: "
                << StringUtil::Quote(_link.getName()));

		return path;
    }

private:
	Trajectory *_trajectory;
	Solver *_solver;
	Link _link;

	Q _current_q;
	State *_current_state;

	static PathPlanner *_path_planner;
	static TrajectoryPlanner *_trajectory_planner;
	static IKMetaSolver *_meta_solver;
};

PathPlanner *LinkVisitor::_path_planner;
TrajectoryPlanner *LinkVisitor::_trajectory_planner;
IKMetaSolver *LinkVisitor::_meta_solver;

Solver::Solver(
    rw::pathplanning::PathPlannerFactory &path_planner_factory,
    rw::pathplanning::TrajectoryPlannerFactory &trajectory_planner_factory)
{
	_path_planner_factory = &path_planner_factory;
	_trajectory_planner_factory = &trajectory_planner_factory;
}

Solver::~Solver() {}

void Solver::Solve(Task &task, rw::kinematics::State &init_state)
{
	_current_state = new State(init_state);

	Task::iterator it;

	Link old_last_link(PropertyMap(), "");
    Link new_first_link(PropertyMap(), "");

	Trajectory *trajectory;
	_first_trajectory = true;

	for (it = task.begin(); it != task.end(); it++) {
		if (trajectory = boost::get<Trajectory>(&*it)) {
			new_first_link = TaskUtil::CombineLinks(old_last_link,*trajectory->link_begin());
			trajectory->link_begin()->setData(new_first_link);
			trajectory->link_begin()->setPrev(old_last_link.prev());
			old_last_link = *(--trajectory->link_end());

			Solve(*trajectory);
		}

		if (Action *action = boost::get<Action>(&*it))
			Solve(*action);
	}
}

void Solver::Solve(Action &action)
{
	if (AttachFrameAction *attach =
        boost::get<AttachFrameAction>(&action.getActionType()))
    {
		Frame *parent = &attach->getParent();
		MovableFrame *child = &attach->getChild();

		Frame *old_parent = child->getParent(*_current_state);
		Transform3D<> ParentTransform = Kinematics::FrameTframe(
            parent,old_parent, *_current_state);
		Transform3D<> oldTransform = child->getTransform(*_current_state);
		child->setTransform(ParentTransform*oldTransform, *_current_state);
		child->attachFrame(*parent, *_current_state);
	}
}

void Solver::Solve(Trajectory &trajectory)
{
	trajectory.storeState(*_current_state);

	Device* device = &trajectory.getDevice();

	_path_planner = _path_planner_factory->make(
        &trajectory.getWorkCell(), device, *_current_state);

	_trajectory_planner = _trajectory_planner_factory->make(
        &trajectory.getWorkCell(), device, *_current_state);

	_meta_solver = make_meta_solver(device);
	LinkVisitor::setPlanners(_path_planner,_trajectory_planner,_meta_solver);

	Trajectory::link_iterator begin;
	if (_first_trajectory) {
		begin = ++trajectory.link_begin();
		_first_trajectory = false;
		Q q_init = calcQ(trajectory,*begin->prev(),*_current_state);
		device->setQ(q_init,*_current_state);
	}
	else {
		begin = trajectory.link_begin();
    }

	Trajectory::link_iterator end = --trajectory.link_end();

	for (Trajectory::link_iterator it = begin; it != end; it++) {
		Link::MotionConstraint motion_contraint = it->getMotionConstraint();

		LinkVisitor link_visitor(&trajectory,*it,_current_state);
		Path path = boost::apply_visitor( link_visitor, motion_contraint);

		it->saveSolvedPath(path);
		if (!path.empty())
			device->setQ(path.back(),*_current_state);
	}
}

rw::math::Q Solver::calcQ(
    const Trajectory &trajectory,
    const Target &target,
    const rw::kinematics::State &state)
{
	if (target.isQ()) return target.getQ();
	else {
		rw::invkin::ResolvedRateSolver r_solver(&trajectory.getDevice());
		vector<Q> res = r_solver.solve(TaskUtil::getBaseTransform(trajectory,target),state);
		if (!res.empty())
			return res.front();

		RW_THROW(
            "Error finding joint configuration for target "
            << StringUtil::Quote(target.getName()));
	}
}
