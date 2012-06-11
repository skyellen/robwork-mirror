#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

void inverseKinematics(Device::Ptr device, const State& state, const Transform3D<>& target)
{
    JacobianIKSolver solver(device, state);
	std::vector<Q> solutions = solver.solve(target, state);
	BOOST_FOREACH(Q q, solutions) {
		std::cout<<"Solution = "<<q<<std::endl;
	}
}
