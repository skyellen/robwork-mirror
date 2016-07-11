#include "ConstantForceManipulator.hpp"
#include "RWBody.hpp"

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rwsim::simulator;

ConstantForceManipulator::ConstantForceManipulator(
		const Vector3D<>& force, std::vector<RWBody*>& bodies):
			_force(force)
{
   BOOST_FOREACH(RWBody* body, bodies){
	   if( body->getType()==RWBody::Rigid ){
		   _bodies.push_back(body);
	   }
   }
}

void ConstantForceManipulator::addForces(rw::kinematics::State &state, double h){
	BOOST_FOREACH(RWBody* body, _bodies){
		body->addGravitationW( _force );
	}
}

