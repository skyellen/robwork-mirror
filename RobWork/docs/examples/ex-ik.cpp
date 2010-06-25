#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/invkin/SimpleSolver.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::invkin;

void inverseKinematics(DevicePtr device, const State& state, const Transform3D<>& target) 
{
	ResolvedRateSolver solver(device, state);
	std::vector<Q> solutions = solver.solve(target, state);
	BOOST_FOREACH(Q q, solutions) {
		std::cout<<"Solution = "<<q<<std::endl;
	}
}
