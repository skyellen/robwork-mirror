/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTConstraint.hpp"

#include "TNTBody.hpp"
#include "TNTFixedBody.hpp"
#include "TNTKinematicBody.hpp"
#include "TNTRigidBody.hpp"

#include "TNTIslandState.hpp"
#include "TNTIntegrator.hpp"

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::tntphysics;

TNTConstraint::TNTConstraint(const TNTBody* parent, const TNTBody* child):
	_parent(parent),
	_child(child)
{
};

TNTConstraint::~TNTConstraint() {
}

const TNTBody* TNTConstraint::getParent() const {
	return _parent;
}

const TNTBody* TNTConstraint::getChild() const {
	return _child;
}

const Vector3D<>& TNTConstraint::getPositionParent() const {
	return _posParent;
}

const Vector3D<>& TNTConstraint::getPositionChild() const {
	return _posChild;
}

const Rotation3D<>& TNTConstraint::getLinearRotationParent() const {
	return _rotLinParent;
}

const Rotation3D<>& TNTConstraint::getLinearRotationChild() const {
	return _rotLinChild;
}

const Rotation3D<>& TNTConstraint::getAngularRotationParent() const {
	return _rotAngParent;
}

const Rotation3D<>& TNTConstraint::getAngularRotationChild() const {
	return _rotAngChild;
}

Vector3D<> TNTConstraint::getPositionParentW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state)*_posParent;
}

Vector3D<> TNTConstraint::getPositionChildW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state)*_posChild;
}
/*
Vector3D<> TNTConstraint::getPositionAvgW(const TNTIslandState &state) const {
	const Vector3D<> p = getPositionParentW(state);
	const Vector3D<> c = getPositionChildW(state);
	return (p+c)/2;
}
*/
Rotation3D<> TNTConstraint::getLinearRotationParentW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotLinParent;
}

Rotation3D<> TNTConstraint::getLinearRotationChildW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotLinChild;
}
/*
Rotation3D<> TNTConstraint::getLinearRotationAvgW(const TNTIslandState &state) const {
	const Rotation3D<> p = getLinearRotationParentW(state);
	const Rotation3D<> c = getLinearRotationChildW(state);
	const EAA<> dif(inverse(p)*c);
	const EAA<> mid(dif.angle()/2.*dif.axis());
	return p*mid.toRotation3D();
}
*/
Rotation3D<> TNTConstraint::getAngularRotationParentW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotAngParent;
}

Rotation3D<> TNTConstraint::getAngularRotationChildW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotAngChild;
}
/*
Rotation3D<> TNTConstraint::getAngularRotationAvgW(const TNTIslandState &state) const {
	const Rotation3D<> p = getAngularRotationParentW(state);
	const Rotation3D<> c = getAngularRotationChildW(state);
	const EAA<> dif(inverse(p)*c);
	const EAA<> mid(dif.angle()/2.*dif.axis());
	return p*mid.toRotation3D();
}
*/
VelocityScrew6D<> TNTConstraint::getVelocityParentW(const TNTIslandState &tntstate, const State& rwstate) const {
	const VelocityScrew6D<> vel = _parent->getVelocityW(rwstate,tntstate);
	const Vector3D<> R = _parent->getWorldTcom(tntstate).P();
	const Vector3D<> pos = getPositionParentW(tntstate);
	const Vector3D<> lin = vel.linear() + cross(vel.angular().angle()*vel.angular().axis(),pos-R);
	return VelocityScrew6D<>(lin,vel.angular());
}

VelocityScrew6D<> TNTConstraint::getVelocityChildW(const TNTIslandState &tntstate, const State& rwstate) const {
	const VelocityScrew6D<> vel = _child->getVelocityW(rwstate,tntstate);
	const Vector3D<> R = _child->getWorldTcom(tntstate).P();
	const Vector3D<> pos = getPositionChildW(tntstate);
	const Vector3D<> lin = vel.linear() + cross(vel.angular().angle()*vel.angular().axis(),pos-R);
	return VelocityScrew6D<>(lin,vel.angular());
}

Wrench6D<> TNTConstraint::getWrench(const TNTIslandState &tntstate) const {
	return tntstate.getWrench(this);
}

Wrench6D<> TNTConstraint::getWrenchApplied(const TNTIslandState &tntstate) const {
	return tntstate.getWrenchApplied(this);
}

Wrench6D<> TNTConstraint::getWrenchConstraint(const TNTIslandState &tntstate) const {
	return tntstate.getWrenchConstraint(this);
}

void TNTConstraint::clearWrench(TNTIslandState &tntstate) {
	tntstate.setWrenchApplied(this,Wrench6D<>());
}

void TNTConstraint::applyWrench(TNTIslandState &tntstate, const Wrench6D<>& wrench) {
	const Wrench6D<> cur = getWrenchApplied(tntstate);
	tntstate.setWrenchApplied(this,cur+wrench);
}

Eigen::VectorXd TNTConstraint::getRHS(double h, const Vector3D<> &gravity, const std::list<TNTConstraint*>& constraints, const State &rwstate, const TNTIslandState &tntstate) const {
	const std::size_t dim = getDimVelocity();
	Eigen::VectorXd res = Eigen::VectorXd::Zero(dim);
	Eigen::Matrix<double, 6, 1> velParent = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> velChild = Eigen::Matrix<double, 6, 1>::Zero();
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(_parent)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(_parent)) {
		const Vector3D<>& R = body->getWorldTcom(tntstate).P();
		const Vector3D<> pos = getPositionParentW(tntstate);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,tntstate);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velParent << lin.e(), ang.e();
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(_parent)) {
		const TNTRigidBody::RigidConfiguration* const config = body->getConfiguration(tntstate);
		const Vector3D<> R = config->getWorldTcom().P();
		const Vector3D<> pos = getPositionParentW(tntstate);
		Vector3D<> Fext = gravity*body->getRigidBody()->getMass()+body->getRigidBody()->getForceW(rwstate);
		Vector3D<> Next = body->getRigidBody()->getTorqueW(rwstate);
		BOOST_FOREACH(TNTConstraint* constraint, constraints) {
			const Wrench6D<> wrench = constraint->getWrenchApplied(tntstate);
			if (constraint->getParent() == body) {
				const Vector3D<> pos = constraint->getPositionParentW(tntstate);
				Fext += wrench.force();
				Next += wrench.torque()+cross(pos-R,wrench.force());
			} else if (constraint->getChild() == body) {
				const Vector3D<> pos = constraint->getPositionChildW(tntstate);
				Fext += -wrench.force();
				Next += -wrench.torque()+cross(pos-R,-wrench.force());
			}
		}
		velParent = body->getIntegrator()->eqPointVelIndependent(pos,h,*config,*config,Fext,Next);
	} else {
		RW_THROW("TNTConstraint (getRHS): the type of body \"" << _parent->get()->getName() << "\" is not supported!");
	}
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(_child)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(_child)) {
		const Vector3D<>& R = body->getWorldTcom(tntstate).P();
		const Vector3D<> pos = getPositionChildW(tntstate);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,tntstate);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velChild << lin.e(), ang.e();
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(_child)) {
		const TNTRigidBody::RigidConfiguration* const config = body->getConfiguration(tntstate);
		const Vector3D<> R = config->getWorldTcom().P();
		const Vector3D<> pos = getPositionChildW(tntstate);
		Vector3D<> Fext = gravity*body->getRigidBody()->getMass()+body->getRigidBody()->getForceW(rwstate);
		Vector3D<> Next = body->getRigidBody()->getTorqueW(rwstate);
		BOOST_FOREACH(TNTConstraint* constraint, constraints) {
			const Wrench6D<> wrench = constraint->getWrenchApplied(tntstate);
			if (constraint->getParent() == body) {
				const Vector3D<> pos = constraint->getPositionParentW(tntstate);
				Fext += wrench.force();
				Next += wrench.torque()+cross(pos-R,wrench.force());
			} else if (constraint->getChild() == body) {
				const Vector3D<> pos = constraint->getPositionChildW(tntstate);
				Fext += -wrench.force();
				Next += -wrench.torque()+cross(pos-R,-wrench.force());
			}
		}
		velChild = body->getIntegrator()->eqPointVelIndependent(pos,h,*config,*config,Fext,Next);
	} else {
		RW_THROW("TNTConstraint (getRHS): the type of body \"" << _child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 1> total = velChild-velParent;

	// Rotate to local constraint coordinates:
	const Rotation3D<> Rlin = getLinearRotationParentW(tntstate);
	const Rotation3D<> Rang = getAngularRotationParentW(tntstate);
	Eigen::Matrix<double,6,1> rotated;
	rotated << inverse(Rlin).e()*total.block(0,0,3,1), inverse(Rang).e()*total.block(3,0,3,1);

	// Now we take only the constraint directions!
	const std::vector<Mode> constraintModes = getConstraintModes();
	std::size_t cI = 0;
	for (std::size_t i = 0; i < 6; i++) {
		const Mode &mode = constraintModes[i];
		if (mode == Velocity) {
			res[cI] = rotated[i];
			cI ++;
		}
	}
	return res;
}

Eigen::MatrixXd TNTConstraint::getLHS(const TNTConstraint* constraint, double h, const State &rwstate, const TNTIslandState &tntstate) const {
	const TNTBody* parent = _parent;
	const TNTBody* child = _child;
	const TNTBody* constraintParent = constraint->getParent();
	const TNTBody* constraintChild = constraint->getChild();
	const std::size_t dim = getDimVelocity();
	Eigen::Matrix<double, 6, 6> matrixParent = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 6> matrixChild = Eigen::Matrix<double, 6, 6>::Zero();
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(parent)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(parent)) {
		// Do nothing
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(parent)) {
		const TNTRigidBody::RigidConfiguration* const config = body->getConfiguration(tntstate);
		const Vector3D<> pos = getPositionParentW(tntstate);
		if (parent == constraintParent) {
			const Vector3D<> constraintPos = constraint->getPositionParentW(tntstate);
			matrixParent = body->getIntegrator()->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (parent == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(tntstate);
			matrixParent = -body->getIntegrator()->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		}
	} else {
		RW_THROW("TNTConstraint (getLHS): one of the bodies \"" << _parent->get()->getName() << "\" and \"" << _child->get()->getName() << "\" must be a TNTRigidBody!");
	}
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(child)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(child)) {
		// Do nothing
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(child)) {
		const TNTRigidBody::RigidConfiguration* const config = body->getConfiguration(tntstate);
		const Vector3D<> pos = getPositionChildW(tntstate);
		if (child == constraintParent) {
			const Vector3D<> constraintPos = constraint->getPositionParentW(tntstate);
			matrixChild = -body->getIntegrator()->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (child == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(tntstate);
			matrixChild = body->getIntegrator()->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		}
	} else {
		RW_THROW("TNTConstraint (getLHS): the type of body \"" << child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 6> total = matrixParent+matrixChild;

	// Now we rotate the constraints and change to force and torque in local coordinates
	const Rotation3D<> Rlin = getLinearRotationParentW(tntstate);
	const Rotation3D<> Rang = getAngularRotationParentW(tntstate);
	const Rotation3D<> RlinB = constraint->getLinearRotationParentW(tntstate);
	const Rotation3D<> RangB = constraint->getAngularRotationParentW(tntstate);
	Eigen::Matrix<double,6,6> Rgen, RgenInv;
	Rgen << inverse(Rlin).e(), Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), inverse(Rang).e();
	RgenInv << RlinB.e(), Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), RangB.e();
	Eigen::Matrix<double,6,6> rotated = Rgen*total*RgenInv;

	// Now we take only the rows and columns for the constrained directions!
	const std::vector<Mode> constraintModes = getConstraintModes();
	const std::vector<Mode> constraintModesB = constraint->getConstraintModes();
	const std::size_t dimB = constraint->getDimVelocity();
	std::size_t cI = 0;
	Eigen::MatrixXd reduced(dim,dimB);
	for (std::size_t i = 0; i < 6; i++) {
		const Mode &modeI = constraintModes[i];
		if (modeI == Velocity) {
			std::size_t cJ = 0;
			for (std::size_t j = 0; j < 6; j++) {
				const Mode &modeJ = constraintModesB[j];
				if (modeJ == Velocity) {
					reduced(cI,cJ) = rotated(i,j);
					cJ++;
				}
			}
			cI++;
		}
	}
	return reduced;
}
