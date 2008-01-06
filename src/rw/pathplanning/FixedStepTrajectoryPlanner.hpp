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

#ifndef rw_pathplanning_FixedStepTrajectoryPlanner_HPP
#define rw_pathplanning_FixedStepTrajectoryPlanner_HPP

/**
 * @file PathPlanner.hpp
 */
#include "TrajectoryPlanner.hpp"
#include <rw/invkin/IterativeIK.hpp>

#include <rw/models/Device.hpp>


namespace rw { namespace pathplanning {

 
	class FixedStepTrajectoryPlanner : public TrajectoryPlanner 
	{

    public:

		FixedStepTrajectoryPlanner(rw::models::Device *device, rw::kinematics::State &state, rw::invkin::IterativeIK &ik, double step_size);

		bool solve(const Q& qInit,
			   const rw::interpolator::Pose6dStraightSegment& interpolator,
               Path& path);


	private:
		rw::models::Device *_device;
		rw::kinematics::State _state;
		rw::invkin::IterativeIK *_ik;
		double _step_size;
  

	};

}} // end namespaces

#endif // end include guard
