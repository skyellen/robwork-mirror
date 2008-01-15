/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_TASK_Solver_HPP
#define RW_TASK_Solver_HPP

/**
 * @file Solver.hpp
 */

#include "Task.hpp"
#include <rw/pathplanning/PathPlanner.hpp>
#include <rw/pathplanning/TrajectoryPlanner.hpp>
#include <rw/pathplanning/PathPlannerFactory.hpp>
#include <rw/pathplanning/TrajectoryPlannerFactory.hpp>

#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/ResolvedRateSolver.hpp>

#include <iostream>
#include <string.h>

namespace rw { namespace task {


	/** @addtogroup task */
    /*@{*/

	class Solver
	{
	public:
		Solver(rw::pathplanning::PathPlannerFactory &path_planner_factory, rw::pathplanning::TrajectoryPlannerFactory &trajectory_planner_factory);

		~Solver();

		bool Solve(Task &task, rw::kinematics::State &init_state);

		bool Solve(Trajectory &trajectory);
		bool Solve(Action &action);

		bool Solve(Link &link);

		rw::pathplanning::PathPlanner *getPathPlanner() { return _path_planner; }
		rw::pathplanning::TrajectoryPlanner *getTrajectoryPlanner() { return _trajectory_planner; }


	private:
		rw::pathplanning::PathPlanner *_path_planner;
		rw::pathplanning::TrajectoryPlanner *_trajectory_planner;

		rw::pathplanning::PathPlannerFactory *_path_planner_factory;
		rw::pathplanning::TrajectoryPlannerFactory *_trajectory_planner_factory;

		rw::invkin::IKMetaSolver *_meta_solver;

		rw::math::Q _current_q;
		rw::kinematics::State *_current_state;

	};


}// end task namespace
}// end rw namespace

#endif
