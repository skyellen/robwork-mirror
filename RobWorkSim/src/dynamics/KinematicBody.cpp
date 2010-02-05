#include "KinematicBody.hpp"

#include <rw/models/Joint.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Q.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace dynamics;
using namespace rw::geometry::sandbox;

namespace {

	ublas::matrix<double> getBlockDiagInertia(Device &dev, std::vector<KinematicBody*>& parents,
											  KinematicBody* current){
		using namespace boost::numeric::ublas;
		size_t n = dev.getDOF();
		matrix<double> A( zero_matrix<double>(n*3,n*3) );
		for(size_t i=0;i<parents.size()+1;i++){
			matrix_range<matrix<double> > Ar (A, range(i*3, i*3+3), range(i*3, i*3+3));
			if(i==parents.size()){
				Ar = current->getInertia().m();
			} else {
				Ar = parents[i]->getInertia().m();
			}
		}
		return A;
	}

}

KinematicBody::KinematicBody(
            const BodyInfo& info,
            rw::kinematics::Frame& frame,
            const std::vector<Geometry*>& geoms,
            rw::kinematics::State& state):
			   Body(info, &frame, geoms),
			   _jointFrame(NULL),
			   _acc(0.0),
			   _vel(0.0),
			   _pos( 0.0 )

{
            	/*
    std::cout << "KinematicBody " ;
    _wTb = rw::kinematics::Kinematics::worldTframe( &_jointFrame, state);
    std::cout << "a";
    _bTw = inverse( _wTb );
    */
}

KinematicBody::~KinematicBody()
{

}


void KinematicBody::rollBack(State &state){
    _torque = _torqueRB;
    _force = _forceRB;

    _linImpulse = Vector3D<>(0.0,0.0,0.0);
    _angImpulse = Vector3D<>(0.0,0.0,0.0);
/*
    _pos = *_jointFrame->getQ(state);
    _wTb = rw::kinematics::Kinematics::WorldTframe( &_jointFrame, state);
    _bTw = inverse( _wTb );

    _baseTb = rw::kinematics::Kinematics::FrameTframe( _base, &_jointFrame, state);
    _bTbase = inverse( _baseTb );

    _wTbase = rw::kinematics::Kinematics::WorldTframe( _base, state);
    _baseTw = inverse( _wTbase );

    _vel = _velRB;
    */
    std::cout << "KinematicBody: ROLLBACK " << _vel << " " << _pos << std::endl;
    //std::cout << _wTb << std::endl;
    //std::cout << _baseTb << std::endl;
    //std::cout << _jac << std::endl;
}

void KinematicBody::saveState(double h, rw::kinematics::State& state){
    // update rollBack variables
    _torqueRB = _torque;
    _forceRB = _force;

    _jointFrame->setQ(state,&_pos);
    _velRB = _vel;
    std::cout << "KinematicBody: SAVESTATE " << _vel << " " << _pos << std::endl;
    //std::cout << _wTb << std::endl;
    //std::cout << _baseTb << std::endl;
    //std::cout << _jac << std::endl;
}

void KinematicBody::resetState(rw::kinematics::State &state){
	_vel = 0;
	_pos = *_jointFrame->getQ(state);

	//std::cout << "KinematicBody: Update position! " << _pos << " " << _vel << " " << h << std::endl;

    _wTb = rw::kinematics::Kinematics::worldTframe( _jointFrame, state);
    _bTw = inverse( _wTb );

    _baseTb = rw::kinematics::Kinematics::frameTframe( _base, _jointFrame, state);
    _bTbase = inverse( _baseTb );

}


rw::math::Vector3D<> KinematicBody::getPointVelW(const rw::math::Vector3D<>& wPp){

    return Vector3D<>(0,0,0);
}


void KinematicBody::updatePosition(double h, State &state){

	// Clamp _vel to minimum 0.0
	_pos += _vel * h;
	_jointFrame->setQ(state, &_pos );

	//std::cout << "KinematicBody: Update position! " << _pos << " " << _vel << " " << h << std::endl;

    _wTb = rw::kinematics::Kinematics::worldTframe( _jointFrame, state);
    _bTw = inverse( _wTb );

    _baseTb = rw::kinematics::Kinematics::frameTframe( _base, _jointFrame, state);
    _bTbase = inverse( _baseTb );

    _wTbase = rw::kinematics::Kinematics::worldTframe( _base, state);
    _baseTw = inverse( _wTbase );


    _impulseIterations = 0;
}

#include <rw/math/LinearAlgebra.hpp>

rw::math::InertiaMatrix<> KinematicBody::getEffectiveMassW(const rw::math::Vector3D<>& wPc){
	using namespace boost::numeric::ublas;
	std::cout << "Eff mass W" << std::endl;
	// calculate the block diagonal inertia matrix of all child bodies
	// the matrix has n blocks where n is number of dof
	// each block is 3x3 inertiamatrix so we get
	/*size_t n = _dev.getDOF();
	matrix<double> A( zero_matrix<double>(n*3,n*3) );
	for(size_t i=0;i<_parents.size()+1;i++){
		matrix_range<matrix<double> > Ar (A, range(i*3, i*3+3), range(i*3, i*3+3));
		if(i==_parents.size()){
			Ar = getInertia().m();
		} else {
			Ar = _parents[i]->getInertia().m();
		}
	}*/

	//matrix<double> A = getBlockDiagInertia(_dev, _parents, this);

	// now calculate the contact jacobian expressed in world coordinates
	//matrix<double> tmp = prod( _jac.m() , A );
	//matrix<double> effInertia = prod( tmp , trans(_jac.m()));
	//return InertiaMatrix<>( effInertia );
	return InertiaMatrix<>( 1,1,1);
}

void KinematicBody::updateImpulse(){
	std::cout << "KinematicBody: Update impulse!" << std::endl;
	ublas::vector<double> cartImpulse(6);
	cartImpulse(0) = _linImpulse(0);
	cartImpulse(1) = _linImpulse(1);
	cartImpulse(2) = _linImpulse(2);
	cartImpulse(3) = _angImpulse(0);
	cartImpulse(4) = _angImpulse(1);
	cartImpulse(5) = _angImpulse(2);

	// too update joint velocities we use: deltaQd = M^-1 J^T P
	//InertiaMatrix<> K = getEffectiveMass();


    _linImpulse = Vector3D<>(0.0,0.0,0.0);
    _angImpulse = Vector3D<>(0.0,0.0,0.0);
    std::cout << "Update Vel END" << std::endl;
}

void KinematicBody::updateVelocity(double h, State &state){
	std::cout << "KinematicBody: Update velocity!" << _jointFrame->getName() << std::endl;
	ublas::vector<double> cartForce(6);
	cartForce(0) = _force(0);
	cartForce(1) = _force(1);
	cartForce(2) = _force(2);
	cartForce(3) = _torque(0);
	cartForce(4) = _torque(1);
	cartForce(5) = _torque(2);


	//_vel += _acc*h + jointTorque();

	double MAX_VEL = 1;
	// clamp the velocity
	if(_vel<-MAX_VEL){
		_vel = -MAX_VEL;
	} else if (_vel>MAX_VEL){
		_vel = MAX_VEL;
	}
}
