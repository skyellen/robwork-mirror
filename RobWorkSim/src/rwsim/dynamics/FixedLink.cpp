/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "FixedLink.hpp"

#include <rw/models/Joint.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Q.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwsim::dynamics;

namespace {

	ublas::matrix<double> getBlockDiagInertia(Device &dev, std::vector<FixedLink*>& parents, 
											  FixedLink* current){
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

FixedLink::FixedLink(
           const BodyInfo& info,
		   rw::kinematics::Frame *base,
		   std::vector<FixedLink*> parents,
		   rw::models::Device &dev, 
		   rw::models::Joint &j,
		   const std::vector<rw::kinematics::Frame*>& frames,
		   rw::kinematics::State &state):
			   Body( ConstraintNode::Link , j, frames),
			   _jointFrame(j),
			   _dev(dev),
			   _parents(parents),
			   _jac( dev.baseJframe(&j, state) ),
			   _jacRB( _jac ),
			   _base(base),
			   _acc(0.0),
			   _vel(0.0),
			   _pos( *(j.getQ(state)) ),
			   _wTbase( dev.worldTbase(state) ),
			   _baseTw( inverse(_wTbase) ),
			   _impulseIterations(0)
			   
{
			   
    _wTb = rw::kinematics::Kinematics::WorldTframe( &_jointFrame, state);
    _bTw = inverse( _wTb );
    
    _baseTb = rw::kinematics::Kinematics::FrameTframe( _base, &_jointFrame, state);
    _bTbase = inverse( _baseTb );	
    
    _wTbase = rw::kinematics::Kinematics::WorldTframe( _base, state);
    _baseTw = inverse( _wTbase );
}

FixedLink::~FixedLink()
{
	
}


void FixedLink::rollBack(State &state){
    _torque = _torqueRB;
    _force = _forceRB;

    _linImpulse = Vector3D<>(0.0,0.0,0.0);
    _angImpulse = Vector3D<>(0.0,0.0,0.0);
    
    _pos = *_jointFrame.getQ(state);
    _wTb = rw::kinematics::Kinematics::WorldTframe( &_jointFrame, state);
    _bTw = inverse( _wTb );
    
    _baseTb = rw::kinematics::Kinematics::FrameTframe( _base, &_jointFrame, state);
    _bTbase = inverse( _baseTb );
    
    _wTbase = rw::kinematics::Kinematics::WorldTframe( _base, state);
    _baseTw = inverse( _wTbase );
    
    _jac = _dev.baseJframe(&_jointFrame, state);
    
    _vel = _velRB;
    std::cout << "FIXEDLINK: ROLLBACK " << _vel << " " << _pos << std::endl;
    //std::cout << _wTb << std::endl;
    //std::cout << _baseTb << std::endl;
    //std::cout << _jac << std::endl;
}

void FixedLink::saveState(double h, rw::kinematics::State& state){
    // update rollBack variables
    _torqueRB = _torque;
    _forceRB = _force;
    
    _jointFrame.setQ(state,&_pos);
    _velRB = _vel;
    std::cout << "FIXEDLINK: SAVESTATE " << _vel << " " << _pos << std::endl;
    //std::cout << _wTb << std::endl;
    //std::cout << _baseTb << std::endl;
    //std::cout << _jac << std::endl;
}

void FixedLink::resetState(rw::kinematics::State &state){
	_vel = 0;
	_pos = *_jointFrame.getQ(state);
	
	//std::cout << "FixedLink: Update position! " << _pos << " " << _vel << " " << h << std::endl;
	
    _wTb = rw::kinematics::Kinematics::WorldTframe( &_jointFrame, state);
    _bTw = inverse( _wTb );
                
    _baseTb = rw::kinematics::Kinematics::FrameTframe( _base, &_jointFrame, state);
    _bTbase = inverse( _baseTb );
    
    _jac = _dev.baseJframe(&_jointFrame, state);
}


void FixedLink::addQd( double vel ){
	double epsilon = 0.00001;
	_vel += vel;
	// TODO: Clamp _vel to minimum [-VELMAX,VELMAX]
	// make sure velocity is no higher than target vel
	if(_targetVel<0 && _vel<_targetVel){
		_vel = _targetVel;
	} else if (_targetVel>0 && _vel>_targetVel){
		_vel = _targetVel;
	}
	
}


rw::math::Vector3D<> FixedLink::getPointVelW(const rw::math::Vector3D<>& wPp){
	rw::math::Q vels( _dev.getDOF() );
	size_t i;
	
	for(i=0;i<_parents.size(); i++)
		vels(i) = _parents[i]->getQd();
	
	vels(i) = _vel;
	for(i++;i<_dev.getDOF();i++)
		vels(i) = 0;
	
	// Velocity of tool seen in base
	rw::math::Rotation3D<> tRb = _bTbase.R(); // tool to base 	
	rw::math::Transform3D<> wTt = _wTbase * _baseTb; // world to tool        	
	rw::math::Transform3D<> pTw = inverse( rw::math::Transform3D<>(wPp, wTt.R()) );	
	rw::math::Transform3D<> pTt = pTw * wTt;
	rw::math::VelocityScrew6D<> bVt = _jac*vels;
	rw::math::VelocityScrew6D<> tVt = tRb * bVt; // change reference frame
	rw::math::VelocityScrew6D<> pVp = pTt.P() * tVt; // change reference point
	rw::math::VelocityScrew6D<> wVp = wTt.R() * pVp; // change reference frame
/*
	std::cout << "tRb " << tRb << std::endl;
	std::cout << "wRt " << wTt.R() << std::endl;
	std::cout << "pPt " << pTt.P() << std::endl;
	std::cout << "Jac: " << _jac.size1() << " " << _jac.size2() << std::endl;
	std::cout << "bVt " << bVt << std::endl;
	std::cout << "tVt " << tVt << std::endl;
	std::cout << "pVp " << pVp << std::endl;
	std::cout << "Vels: " << vels << std::endl;
	std::cout << "wVp: " << wVp << std::endl;
*/
	// Change the base of reference by multiplying with the inverserotation
	return wVp.linear();
}


void FixedLink::updatePosition(double h, State &state){
	
	// Clamp _vel to minimum 0.0 	
	_pos += _vel * h;
	_jointFrame.setQ(state, &_pos );
	
	//std::cout << "FixedLink: Update position! " << _pos << " " << _vel << " " << h << std::endl;
	
    _wTb = rw::kinematics::Kinematics::WorldTframe( &_jointFrame, state);
    _bTw = inverse( _wTb );
     
    _baseTb = rw::kinematics::Kinematics::FrameTframe( _base, &_jointFrame, state);
    _bTbase = inverse( _baseTb );

    _wTbase = rw::kinematics::Kinematics::WorldTframe( _base, state);
    _baseTw = inverse( _wTbase );
    
    _jac = _dev.baseJframe(&_jointFrame, state);
    
    _impulseIterations = 0;
}

#include <rw/math/LinearAlgebra.hpp>

rw::math::InertiaMatrix<> FixedLink::getEffectiveMassW(const rw::math::Vector3D<>& wPc){
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
	
	matrix<double> A = getBlockDiagInertia(_dev, _parents, this);
	
	// now calculate the contact jacobian expressed in world coordinates
	matrix<double> tmp = prod( _jac.m() , A );
	matrix<double> effInertia = prod( tmp , trans(_jac.m()));
	return InertiaMatrix<>( effInertia );
}

rw::math::InertiaMatrix<> FixedLink::getEffectiveMass(){
	using namespace boost::numeric::ublas;
	std::cout << "Eff mass";
	// calculate the block diagonal inertia matrix of all child bodies
	// the matrix has n blocks where n is number of dof
	// each block is 3x3 inertiamatrix so we get
	/*
	size_t n = _dev.getDOF();
	matrix<double> A( zero_matrix<double>(n*3,n*3) );
	for(size_t i=0;i<_parents.size()+1;i++){
		matrix_range<matrix<double> > Ar (A, range(i*3, i*3+3), range(i*3, i*3+3));
		if(i==_parents.size()){
			Ar = getInertia().m();
		} else {
			Ar = _parents[i]->getInertia().m();
		}
	}
	*/
	matrix<double> A = getBlockDiagInertia(_dev, _parents, this);
	
	// now calculate the effective mass inertia seen in the joint 
	matrix<double> tmp = prod( _jac.m() , A );
	matrix<double> effInertia = prod( tmp , trans(_jac.m()));
	std::cout << " ! " << std::endl;
	return InertiaMatrix<>( effInertia );
}

void FixedLink::updateImpulse(){
	std::cout << "FixedLink: Update impulse!" << std::endl;
	ublas::vector<double> cartImpulse(6);
	cartImpulse(0) = _linImpulse(0);
	cartImpulse(1) = _linImpulse(1);
	cartImpulse(2) = _linImpulse(2);
	cartImpulse(3) = _angImpulse(0);
	cartImpulse(4) = _angImpulse(1);
	cartImpulse(5) = _angImpulse(2);
	
	// too update joint velocities we use: deltaQd = M^-1 J^T P 
	//InertiaMatrix<> K = getEffectiveMass();
	
	std::cout << "get diag inertia" << std::endl;
	ublas::matrix<double> A = getBlockDiagInertia(_dev, _parents, this);
	std::cout << "Calc JT_P" << std::endl;
	ublas::vector<double> JT_P =  prod( trans(_jac.m()) , cartImpulse );
	std::cout << "Calc jointVel" << std::endl;
	ublas::vector<double> jointVel = prod( LinearAlgebra::pseudoInverse(A) , JT_P );  
	//ublas::vector<double> jointVel = prod( LinearAlgebra::PseudoInverse(_jac.m()) , cartImpulse ) *0.001 ;
	// add the velocity to all joints that is affected
	std::cout << "Add vel to parents: " 
			  << " " << _parents.size() 
			  << " " << jointVel.size()
			  << std::endl;
	
	for(size_t i = 0; i<_parents.size(); i++){
		_parents[i]->addQd( jointVel(i) );
	}
	addQd( jointVel(_parents.size()) );
	
    _linImpulse = Vector3D<>(0.0,0.0,0.0);
    _angImpulse = Vector3D<>(0.0,0.0,0.0);
    std::cout << "Update Vel END" << std::endl;
}

void FixedLink::updateVelocity(double h, State &state){
	std::cout << "FixedLink: Update velocity!" << _jointFrame.getName() << std::endl;
	ublas::vector<double> cartForce(6);
	cartForce(0) = _force(0);
	cartForce(1) = _force(1);
	cartForce(2) = _force(2);
	cartForce(3) = _torque(0);
	cartForce(4) = _torque(1);
	cartForce(5) = _torque(2);
	ublas::vector<double> tmp = prod( trans(_jac.m()) , cartForce );
	ublas::vector<double> jointTorque = prod(_jac.m(), tmp ) * h;
	//std::cout << "Joint Torque: " << jointTorque << std::endl;
	for(size_t i = 0; i<_parents.size(); i++){
		//std::cout << "Adding to parent " << i << std::endl; 
		_parents[i]->addQd( jointTorque(i) );
	}
	//std::cout << "Adding FORCE : " << _acc*h << " and " << jointTorque(_parents.size()) << std::endl;
	addQd( jointTorque(_parents.size()) + _acc*h );
	
	//_vel += _acc*h + jointTorque();
	
	double MAX_VEL = 1;
	// clamp the velocity
	if(_vel<-MAX_VEL){
		_vel = -MAX_VEL;
	} else if (_vel>MAX_VEL){
		_vel = MAX_VEL;
	}
}

void FixedLink::addForceWToPosW(const rw::math::Vector3D<>& force, 
                             const rw::math::Vector3D<>& pos){
    
    // transform the force into body frame description
    rw::math::Vector3D<> forcebody = _baseTw.R() * force;
    
    // calculate the center force contribution
    _force += forcebody;    
    rw::math::Vector3D<> posOnBody = _baseTw.R() * (pos - _wTb.P());
    
    // calculate the torque contribution
    _torque += cross( posOnBody, forcebody );
}

void FixedLink::addImpulseWToPosW(const rw::math::Vector3D<>& impulse, 
                       const rw::math::Vector3D<>& pos){
	
	//std::cout << "FixedLink: Impulse at: " << pos << std::endl;
	// transform the force into body frame description
    rw::math::Vector3D<> ibody = _baseTw.R() * impulse;
    
    // calculate the center force contribution
    _linImpulse += ibody;
    rw::math::Vector3D<> posOnBody = _baseTw.R() * (pos - _wTb.P());

    // calculate the torque contribution
    _angImpulse += cross( posOnBody , ibody );
}
