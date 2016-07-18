#include <rw/invkin/JacobianIKSolver.hpp>

using rw::invkin::JacobianIKSolver;
using rw::kinematics::State;
using namespace rw::math;
using rw::models::Device;

void inverseKinematics(rw::common::Ptr<Device> device, const State& state, const Transform3D<>& target)
{
    JacobianIKSolver solver(device, state);
	std::vector<Q> solutions = solver.solve(target, state);
	BOOST_FOREACH(Q q, solutions) {
		std::cout<<"Solution = "<<q<<std::endl;
	}
}
