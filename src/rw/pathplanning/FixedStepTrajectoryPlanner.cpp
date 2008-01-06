#include "FixedStepTrajectoryPlanner.hpp"

#include <rw/math/Math.hpp>

using namespace rw::pathplanning;
using namespace rw::math;


FixedStepTrajectoryPlanner::FixedStepTrajectoryPlanner(rw::models::Device *device, rw::kinematics::State &state, rw::invkin::IterativeIK &ik, double step_size)
{
	_device = device;
	_state = state;
	_ik = &ik;
	_step_size = step_size;


}


bool FixedStepTrajectoryPlanner::solve(const Q& qInit, const rw::interpolator::Pose6dStraightSegment& interpolator, Path& path)
{
	std::pair<double, double> interval = interpolator.getInterval();
	
	_device->setQ(qInit, _state);

	std::cout << "start" << std::endl;
		
	for(double t = interval.first; t <= interval.second; t+= _step_size) {
		std::vector<Q> res = _ik->solve(interpolator.getX(t),_state);

		if(res.empty())
			return false;

		Q q = res.front();
		path.push_back(q);
		_device->setQ(q, _state);
		

	}

	return true;
}