#include "Solver.hpp"

using namespace rw::task;
using namespace std;

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;
using namespace rw::pathplanning;
using namespace rw::math;

#include "TaskUtil.hpp"

#include <rw/kinematics/Kinematics.hpp>

rw::invkin::IKMetaSolver *make_meta_solver(rw::models::Device *device)
{
	return new rw::invkin::IKMetaSolver(new rw::invkin::ResolvedRateSolver(device),device,NULL);

}



class LinkVisitor : public boost::static_visitor<Path>
{
public:

	LinkVisitor(Trajectory *trajectory, Link &link, Q &current_q, State *current_state)
		: _trajectory(trajectory), _link(link), _current_q(current_q), _current_state(current_state)
	{}

	static void setPlanners(rw::pathplanning::PathPlanner *path_planner, TrajectoryPlanner *trajectory_planner, IKMetaSolver *meta_solver)
	{	
		_meta_solver = meta_solver;
		_path_planner = path_planner;
		_trajectory_planner = trajectory_planner;
	}

    Path operator()(NoConstraint i)
    {
		Path path;

		if(_link.next()->isQ())
			_status = _path_planner->query(_current_q,_link.next()->getQ(),path,60);
		else {
			_trajectory->getDevice()->setQ(_current_q,*_current_state);
			vector<Q> res =_meta_solver->solve(TaskUtil::getBaseTransform(*_trajectory,*_link.next()),*_current_state);
			_status = _path_planner->query(_current_q,res.front(),path,60);
		}

		return path;

    }

    Path operator()(LinearJointConstraint i)
    {
		Path path;
		path.push_back(_current_q);

		if(_link.next()->isQ())
			path.push_back(_link.next()->getQ());
		else {
			_trajectory->getDevice()->setQ(_current_q,*_current_state);
			vector<Q> res =_meta_solver->solve(TaskUtil::getBaseTransform(*_trajectory,*_link.next()),*_current_state);
			path.push_back(res.front());
		}

		return path;
    }

	Path operator()(LinearToolConstraint i)
    {
		Path path;
		_status = _trajectory_planner->solve(_current_q,TaskUtil::getPoseInterpolator(*_trajectory,_link),path);

		return path;

    }

	Path operator()(CircularToolConstraint i)
    {
		Path path;
		_status = _trajectory_planner->solve(_current_q,TaskUtil::getPoseInterpolator(*_trajectory,_link),path);
		return path;

    }

	bool getStatus() { return _status; }

private:
	bool _status;

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


Solver::Solver(rw::pathplanning::PathPlannerFactory &path_planner_factory, rw::pathplanning::TrajectoryPlannerFactory &trajectory_planner_factory)
{
	_path_planner_factory = &path_planner_factory;
	_trajectory_planner_factory = &trajectory_planner_factory;

}

Solver::~Solver()
{

}



bool Solver::Solve(Task &task, rw::kinematics::State &init_state)
{
	_current_state = new State(init_state);

	Task::iterator it;

	for(it = task.begin(); it != task.end(); it++) {
		if(Trajectory *trajectory = boost::get<Trajectory>(&*it))
			if(!Solve(*trajectory))
				return false;

		if(Action *action = boost::get<Action>(&*it))
			if(!Solve(*action))
				return false;

	}

	return true;

}

bool Solver::Solve(Action &action)
{
	if(AttachFrameAction *attach = boost::get<AttachFrameAction>(&action.getActionType())) {
		Frame *parent = attach->getParent();
		MovableFrame *child = attach->getChild();

		Frame *old_parent = child->getParent(*_current_state);
		Transform3D<> ParentTransform = Kinematics::FrameTframe(parent,old_parent, *_current_state);
		Transform3D<> oldTransform = child->getTransform(*_current_state);
		child->setTransform(ParentTransform*oldTransform, *_current_state);
		child->attachFrame(*parent, *_current_state);
	}


	return true;
}



bool Solver::Solve(Trajectory &trajectory)
{
	trajectory.storeState(*_current_state);

	Device *device = trajectory.getDevice();

	_path_planner = _path_planner_factory->make(trajectory.getWorkCell(),device,*_current_state);
	_trajectory_planner = _trajectory_planner_factory->make(trajectory.getWorkCell(),device,*_current_state);
	_meta_solver = make_meta_solver(device);
	LinkVisitor::setPlanners(_path_planner,_trajectory_planner,_meta_solver);

	Trajectory::link_iterator it;
	Trajectory::link_iterator begin = ++trajectory.link_begin();
	Trajectory::link_iterator end = --trajectory.link_end();

	for(it = begin; it != end; it++) {

		if(it->prev()->isQ())
			_current_q = it->prev()->getQ();

		Link::MotionConstraint motion_contraint = it->getMotionConstraint();

		LinkVisitor link_visitor(&trajectory,*it,_current_q,_current_state);
		Path path = boost::apply_visitor( link_visitor, motion_contraint);

		it->saveSolvedPath(path);
		_current_q = path.back();
	}

	device->setQ(_current_q,*_current_state);

	return true;
}

