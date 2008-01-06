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

#include <iostream>
#include <string.h>

namespace rw { namespace task {


	/** @addtogroup task */
    /*@{*/




	class Solver
	{
	public:
		Solver(rw::pathplanning::PathPlanner &path_planner, rw::pathplanning::TrajectoryPlanner &trajectory_planner);

		~Solver();

		bool Solve(Task &task);

		bool Solve(Trajectory &trajectory);

		bool Solve(Link &link);

		rw::pathplanning::PathPlanner *getPathPlanner() { return _path_planner; }
		rw::pathplanning::TrajectoryPlanner *getTrajectoryPlanner() { return _trajectory_planner; }


	private:
		rw::pathplanning::PathPlanner *_path_planner;
		rw::pathplanning::TrajectoryPlanner *_trajectory_planner;

		rw::math::Q qCurrent;


	};


}// end task namespace
}// end rw namespace

#endif
