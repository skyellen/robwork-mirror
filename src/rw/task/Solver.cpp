#include "Solver.hpp"

using namespace rw::task;
using namespace std;

using namespace rw::pathplanning;
using namespace rw::math;

#include "TaskUtil.hpp"

class LinkVisitor : public boost::static_visitor<Path>
{
public:

	LinkVisitor(Trajectory *trajectory, Link &link, Q &qCurrent)
		: _trajectory(trajectory), _link(link), _qCurrent(qCurrent)
	{}

	static void setPlanners(PathPlanner *path_planner, TrajectoryPlanner *trajectory_planner) 
	{
		_path_planner = path_planner;
		_trajectory_planner = trajectory_planner;
	}
		
    Path operator()(NoConstraint i)
    {	
		Path path;

		assert(_link.next()->isQ());
		_status = _path_planner->query(_qCurrent,_link.next()->getQ(),path,60);
		return path;

    }

    Path operator()(LinearJointConstraint i)
    {
		Path path;
		path.push_back(_qCurrent);
		assert(_link.next()->isQ());
		path.push_back(_link.next()->getQ());

		return path;
    }
	  
	Path operator()(LinearToolConstraint i)
    {
		Path path;
		_status = _trajectory_planner->solve(_qCurrent,TaskUtil::getPoseInterpolator(*_trajectory,_link),path);
		return path;

    }

	Path operator()(CircularToolConstraint i)
    {
		Path path;
		_status = _trajectory_planner->solve(_qCurrent,TaskUtil::getPoseInterpolator(*_trajectory,_link),path);
		return path;

    }

	bool getStatus() { return _status; }

private:
	bool _status;

	Trajectory *_trajectory;
	Solver *_solver;
	Link _link;

	Q _qCurrent;

	static rw::pathplanning::PathPlanner *_path_planner;
	static rw::pathplanning::TrajectoryPlanner *_trajectory_planner;


};	

rw::pathplanning::PathPlanner *LinkVisitor::_path_planner;
rw::pathplanning::TrajectoryPlanner *LinkVisitor::_trajectory_planner;



Solver::Solver(rw::pathplanning::PathPlanner &path_planner, rw::pathplanning::TrajectoryPlanner &trajectory_planner)
{
	_path_planner = &path_planner;
	_trajectory_planner = &trajectory_planner;

	LinkVisitor::setPlanners(_path_planner,_trajectory_planner);

}

Solver::~Solver()
{
	
}



bool Solver::Solve(Task &task)
{
	Task::iterator it;
	
	for(it = task.begin(); it != task.end(); it++) {


		if(Trajectory *trajectory = boost::get<Trajectory>(&*it))
			if(!Solve(*trajectory))
				return false;

	}

	return true;

}

bool Solver::Solve(Trajectory &trajectory)
{
	Trajectory::link_iterator it;
	Trajectory::link_iterator begin = ++trajectory.link_begin();
	Trajectory::link_iterator end = --trajectory.link_end(); 

	for(it = begin; it != end; it++) {

		if(it->prev()->isQ()) 
			qCurrent = it->prev()->getQ();

		Link::MotionConstraint motion_contraint = it->getMotionConstraint();
		
		LinkVisitor link_visitor(&trajectory,*it,qCurrent);
		Path path = boost::apply_visitor( link_visitor, motion_contraint);

		it->saveSolvedPath(path);
		qCurrent = path.back();


	}
	
	return true;
}
