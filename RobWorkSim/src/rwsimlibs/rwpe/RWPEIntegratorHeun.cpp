/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rwsim/dynamics/RigidBody.hpp>
#include "RWPEIntegratorEuler.hpp"
#include "RWPEIntegratorHeun.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEIntegratorHeun::RWPEIntegratorHeun():
	RWPEIntegrator(NULL),
	_euler(new RWPEIntegratorEuler())
{
}

RWPEIntegratorHeun::RWPEIntegratorHeun(const RWPEBodyDynamic* body):
	RWPEIntegrator(body),
	_euler(new RWPEIntegratorEuler(body))
{
}

RWPEIntegratorHeun::~RWPEIntegratorHeun() {
	delete _euler;
}

const RWPEIntegrator* RWPEIntegratorHeun::makeIntegrator(const RWPEBodyDynamic* body) const {
	return new RWPEIntegratorHeun(body);
}

void RWPEIntegratorHeun::integrate(const Wrench6D<> &netFT, double h, RWPEBodyDynamic::RigidConfiguration &configuration) const {
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (integrate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	RW_THROW("Not working yet!");
}

void RWPEIntegratorHeun::positionUpdate(const Wrench6D<> &netFT, double h, RWPEBodyDynamic::RigidConfiguration &configuration) const {
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (positionUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	// Extract required info
	const rw::common::Ptr<const RigidBody> rwbody = getBody()->getRigidBody();
	const Transform3D<>& wTb = configuration.getWorldTcom();
	const Vector3D<>& R = wTb.P();
	const VelocityScrew6D<>& vel = configuration.getVelocity();
	const InertiaMatrix<>& inertia = wTb.R()*rwbody->getBodyInertia()*inverse(wTb.R());
	const InertiaMatrix<>& inertiaInv = wTb.R()*rwbody->getBodyInertiaInv()*inverse(wTb.R());
	const double massInv = rwbody->getMassInv();
	const Vector3D<> velAng(vel[3],vel[4],vel[5]);// = vel.angular().axis()*vel.angular().angle();

	// Velocity prediction
	const Vector3D<> linVelPred = vel.linear()+h*massInv*netFT.force();
	const Vector3D<> angVelPred = velAng + inertiaInv*(-cross(velAng,inertia*velAng)+netFT.torque())*h;

	// Find new position and orientation
	const Vector3D<> linPos = R+h/2*(vel.linear() + linVelPred);
	const Rotation3D<> angPos = EAA<>(h/2*(velAng+angVelPred)).toRotation3D()*wTb.R();
	configuration.setWorldTcom(Transform3D<>(linPos,angPos));
	configuration.setVelocity(VelocityScrew6D<>(linVelPred,EAA<>(angVelPred)));
}
/*
void RWPEIntegratorHeun::velocityPrediction(const Wrench6D<> &netFT, double h, RWPEBodyDynamic::RigidConfiguration &configuration) const {
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (velocityPrediction): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	// Extract required info
	const rw::common::Ptr<const RigidBody> rwbody = getBody()->getRigidBody();
	const Transform3D<>& wTb = configuration.getWorldTcom();
	const Vector3D<>& R = wTb.P();
	const VelocityScrew6D<>& vel = configuration.getVelocity();
	const InertiaMatrix<>& inertia = wTb.R()*rwbody->getBodyInertia()*inverse(wTb.R());
	const InertiaMatrix<>& inertiaInv = wTb.R()*rwbody->getBodyInertiaInv()*inverse(wTb.R());
	const double massInv = rwbody->getMassInv();
	const Vector3D<> velAng(vel[3],vel[4],vel[5]);// = vel.angular().axis()*vel.angular().angle();

	// Velocity prediction
	const Vector3D<> linVelPred = vel.linear()+h*massInv*netFT.force();
	const Vector3D<> angVelPred = velAng + inertiaInv*(-cross(velAng,inertia*velAng)+netFT.torque())*h;
	configuration.setVelocity(VelocityScrew6D<>(linVelPred,EAA<>(angVelPred)));
}*/

void RWPEIntegratorHeun::velocityUpdate(const Wrench6D<> &netFTcur, const Wrench6D<> &netFTnext, double h, const RWPEBodyDynamic::RigidConfiguration &configuration0, RWPEBodyDynamic::RigidConfiguration &configurationH, class RWPELogUtil& log) const {
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (velocityUpdate): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	// Extract required info
	const rw::common::Ptr<const RigidBody> rwbody = getBody()->getRigidBody();
	const double massInv = rwbody->getMassInv();

	const Transform3D<>& wTb_0 = configuration0.getWorldTcom();
	const VelocityScrew6D<>& vel_0 = configuration0.getVelocity();
	const InertiaMatrix<>& inertia_0 = wTb_0.R()*rwbody->getBodyInertia()*inverse(wTb_0.R());
	const InertiaMatrix<>& inertiaInv_0 = wTb_0.R()*rwbody->getBodyInertiaInv()*inverse(wTb_0.R());
	const Vector3D<> velAng_0(vel_0[3],vel_0[4],vel_0[5]);// = vel.angular().axis()*vel.angular().angle();

	const Transform3D<>& wTb_H = configurationH.getWorldTcom();
	const VelocityScrew6D<>& vel_H = configurationH.getVelocity();
	const InertiaMatrix<>& inertia_H = wTb_H.R()*rwbody->getBodyInertia()*inverse(wTb_H.R());
	const InertiaMatrix<>& inertiaInv_H = wTb_H.R()*rwbody->getBodyInertiaInv()*inverse(wTb_H.R());
	const Vector3D<> velAng_H(vel_H[3],vel_H[4],vel_H[5]);// = vel.angular().axis()*vel.angular().angle();

	// Find new velocity
	const Vector3D<> linVel = vel_0.linear() + h/2*massInv*(netFTcur.force()+netFTnext.force());
	//std::cout << "linVel: " << linVel << " " << vel_0.linear() << " " << vel_H.linear() << " " << netFTcur.force() << " " << netFTnext.force() << std::endl;
	const Vector3D<> angVel = velAng_0 - (inertiaInv_0*cross(velAng_0,inertia_0*velAng_0) + inertiaInv_H*cross(velAng_H,inertia_H*velAng_H))*h/2+(inertiaInv_0*netFTcur.torque()+inertiaInv_H*netFTnext.torque())*h/2;
	configurationH.setVelocity(VelocityScrew6D<>(linVel,EAA<>(angVel)));
}

Eigen::Matrix<double, 6, 1> RWPEIntegratorHeun::eqPointVelIndependent(
	const Vector3D<> point, double h,
	const RWPEBodyDynamic::RigidConfiguration &configuration0,
	const RWPEBodyDynamic::RigidConfiguration &configurationH,
	const Vector3D<>& Ftot0, const Vector3D<>& Ntot0,
	const Vector3D<>& FextH, const Vector3D<>& NextH) const
{
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (eqPointVelIndependent): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const rw::common::Ptr<const RigidBody> rwbody = getBody()->getRigidBody();
	const Transform3D<>& wTb0 = configuration0.getWorldTcom();
	const Transform3D<>& wTbH = configurationH.getWorldTcom();
	const Vector3D<>& RH = wTbH.P();
	const VelocityScrew6D<>& vel0 = configuration0.getVelocity();
	const VelocityScrew6D<>& velH = configurationH.getVelocity();
	const InertiaMatrix<>& inertia0 = wTb0.R()*rwbody->getBodyInertia()*inverse(wTb0.R());
	const InertiaMatrix<>& inertiaInv0 = wTb0.R()*rwbody->getBodyInertiaInv()*inverse(wTb0.R());
	const InertiaMatrix<>& inertiaH = wTbH.R()*rwbody->getBodyInertia()*inverse(wTbH.R());
	const InertiaMatrix<>& inertiaInvH = wTbH.R()*rwbody->getBodyInertiaInv()*inverse(wTbH.R());
	const double massInv = rwbody->getMassInv();
	const Vector3D<> velAng0(vel0[3],vel0[4],vel0[5]);
	const Vector3D<> velAngH(velH[3],velH[4],velH[5]);

	const Vector3D<> angVel = velAng0+h/2.*(inertiaInv0*Ntot0+inertiaInvH*NextH)-h/2.*(inertiaInv0*cross(velAng0,inertia0*velAng0)+inertiaInvH*cross(velAngH,inertiaH*velAngH));
	const Vector3D<> linVel = vel0.linear() + h/2.*massInv*(Ftot0+FextH) + cross(angVel,point-RH);

	Eigen::Matrix<double, 6, 1> a;
	a << linVel.e(), angVel.e();
	return a;
}

Eigen::Matrix<double, 6, 6> RWPEIntegratorHeun::eqPointVelConstraintWrenchFactor(const Vector3D<> point, double h, const Vector3D<> constraintPos, const RWPEBodyDynamic::RigidConfiguration &configuration, const RWPEBodyDynamic::RigidConfiguration &configurationGuess) const {
	if (getBody() == NULL)
		RW_THROW("RWPEIntegratorHeun (eqPointVelConstraintWrenchFactor): There is no body set for this integrator - please construct a new integrator for the specific body to use.");
	const rw::common::Ptr<const RigidBody> rwbody = getBody()->getRigidBody();
	const Transform3D<>& wTb = configuration.getWorldTcom();
	const Vector3D<>& R = wTb.P();
	const Eigen::Matrix3d inertiaInv = (wTb.R()*rwbody->getBodyInertiaInv()*inverse(wTb.R())).e();
	const double massInv = rwbody->getMassInv();
	const double step = h/2.;
	const Eigen::Matrix3d BLinTorque = -step*Math::skew(point-R)*inertiaInv;
	const Eigen::Matrix3d BLinForce = BLinTorque*Math::skew(constraintPos-R)+step*massInv*Rotation3D<>::identity().e();
	const Eigen::Matrix3d BAngTorque = step*inertiaInv;
	const Eigen::Matrix3d BAngForce = BAngTorque*Math::skew(constraintPos-R);
	Eigen::Matrix<double, 6, 6> B;
	B <<	BLinForce, BLinTorque,
			BAngForce, BAngTorque;
	return B;
}

bool RWPEIntegratorHeun::eqIsApproximation() const {
	return false;
}

const RWPEIntegrator* RWPEIntegratorHeun::getDiscontinuityIntegrator() const {
	return _euler;
}
