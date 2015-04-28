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

Rotation3D<> TNTConstraint::getLinearRotationParentW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotLinParent;
}

Rotation3D<> TNTConstraint::getLinearRotationParentForceW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotLinParent;
}

Rotation3D<> TNTConstraint::getLinearRotationChildW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotLinChild;
}

Rotation3D<> TNTConstraint::getLinearRotationChildForceW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotLinChild;
}

Rotation3D<> TNTConstraint::getAngularRotationParentW(const TNTIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotAngParent;
}

Rotation3D<> TNTConstraint::getAngularRotationChildW(const TNTIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotAngChild;
}

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
	clearWrenchApplied(tntstate);
	clearWrenchConstraint(tntstate);
}

void TNTConstraint::clearWrenchApplied(TNTIslandState &tntstate) {
	tntstate.setWrenchApplied(this,Wrench6D<>());
}

void TNTConstraint::clearWrenchConstraint(TNTIslandState &tntstate) {
	tntstate.setWrenchConstraint(this,Wrench6D<>());
}

void TNTConstraint::applyWrench(TNTIslandState &tntstate, const Wrench6D<>& wrench) {
	const Wrench6D<> cur = getWrenchApplied(tntstate);
	tntstate.setWrenchApplied(this,cur+wrench);
}

Eigen::VectorXd TNTConstraint::getRHS(double h, const Vector3D<> &gravity, bool discontinuity, const std::list<TNTConstraint*>& constraints0, const std::list<TNTConstraint*>& constraintsH, const State &rwstate, const TNTIslandState &tntstate0, const TNTIslandState &tntstateH) const {
	const std::size_t dim = 6-getDimFree();
	RW_ASSERT(dim >= 0);
	Eigen::VectorXd res = Eigen::VectorXd::Zero(dim);
	Eigen::Matrix<double, 6, 1> velParent = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> velChild = Eigen::Matrix<double, 6, 1>::Zero();
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(_parent)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(_parent)) {
		const Vector3D<>& R = body->getWorldTcom(tntstateH).P();
		const Vector3D<> pos = getPositionParentW(tntstateH);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,tntstate0);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velParent << lin.e(), ang.e();
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(_parent)) {
		const TNTRigidBody::RigidConfiguration* const config0 = body->getConfiguration(tntstate0);
		const TNTRigidBody::RigidConfiguration* const configH = body->getConfiguration(tntstateH);
		const Vector3D<> pos = getPositionParentW(tntstateH);
		const Wrench6D<> Wtot0 = body->getNetWrench(gravity,constraints0,tntstate0,rwstate);
		const Wrench6D<> WextH = body->getExternalWrench(gravity,constraintsH,tntstateH,rwstate);
		const TNTIntegrator* integrator = body->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		velParent = integrator->eqPointVelIndependent(pos,h,*config0,*configH,Wtot0.force(),Wtot0.torque(),WextH.force(),WextH.torque());
	} else {
		RW_THROW("TNTConstraint (getRHS): the type of body \"" << _parent->get()->getName() << "\" is not supported!");
	}
	if (const TNTFixedBody* const body = dynamic_cast<const TNTFixedBody*>(_child)) {
		// Do nothing
	} else if (const TNTKinematicBody* const body = dynamic_cast<const TNTKinematicBody*>(_child)) {
		const Vector3D<>& R = body->getWorldTcom(tntstateH).P();
		const Vector3D<> pos = getPositionChildW(tntstateH);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,tntstate0);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velChild << lin.e(), ang.e();
	} else if (const TNTRigidBody* const body = dynamic_cast<const TNTRigidBody*>(_child)) {
		const TNTRigidBody::RigidConfiguration* const config0 = body->getConfiguration(tntstate0);
		const TNTRigidBody::RigidConfiguration* const configH = body->getConfiguration(tntstateH);
		const Vector3D<> pos = getPositionChildW(tntstateH);
		const Wrench6D<> Wtot0 = body->getNetWrench(gravity,constraints0,tntstate0,rwstate);
		const Wrench6D<> WextH = body->getExternalWrench(gravity,constraintsH,tntstateH,rwstate);
		const TNTIntegrator* integrator = body->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		velChild = integrator->eqPointVelIndependent(pos,h,*config0,*configH,Wtot0.force(),Wtot0.torque(),WextH.force(),WextH.torque());
	} else {
		RW_THROW("TNTConstraint (getRHS): the type of body \"" << _child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 1> total = velChild-velParent;

	// Rotate to local constraint coordinates:
	const Rotation3D<> Rlin = getLinearRotationParentW(tntstateH);
	const Rotation3D<> Rang = getAngularRotationParentW(tntstateH);
	Eigen::Matrix<double,6,1> rotated;
	rotated << inverse(Rlin).e()*total.block(0,0,3,1), inverse(Rang).e()*total.block(3,0,3,1);

	// Now we take only the constraint directions!
	const std::vector<Mode> constraintModes = getConstraintModes();
	std::size_t cI = 0;
	bool wrenchConstraints = false;
	Eigen::VectorXd wrenchRHS;
	std::size_t iWrench = 0;
	for (std::size_t i = 0; i < 6; i++) {
		const Mode &mode = constraintModes[i];
		if (mode == Velocity) {
			res[cI] = rotated[i];
			cI ++;
		} else if (mode == Wrench) {
			if (!wrenchConstraints) {
				wrenchConstraints = true;
				wrenchRHS = getWrenchModelRHS();
				RW_ASSERT(wrenchRHS.rows() == (int)getDimWrench());
			}
			res[cI] = wrenchRHS[iWrench];
			iWrench++;
			cI ++;
		}
	}
	return res;
}

Eigen::MatrixXd TNTConstraint::getLHS(const TNTConstraint* constraint, double h, bool discontinuity, const State &rwstate, const TNTIslandState &tntstate) const {
	const TNTBody* parent = _parent;
	const TNTBody* child = _child;
	const TNTBody* constraintParent = constraint->getParent();
	const TNTBody* constraintChild = constraint->getChild();
	const std::size_t dim = getDimVelocity()+getDimWrench();
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
			const TNTIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixParent = integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (parent == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(tntstate);
			const TNTIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixParent = -integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
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
			const TNTIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixChild = -integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (child == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(tntstate);
			const TNTIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixChild = integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		}
	} else {
		RW_THROW("TNTConstraint (getLHS): the type of body \"" << child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 6> total = matrixParent+matrixChild;

	// Now we rotate the constraints and change to force and torque in local coordinates
	const Rotation3D<> Rlin = getLinearRotationParentW(tntstate);
	const Rotation3D<> Rang = getAngularRotationParentW(tntstate);
	const Rotation3D<> RlinB = constraint->getLinearRotationParentForceW(tntstate);
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
	//const std::size_t dimB = (constraint->getDimVelocity()+constraint->getDimWrench() > 0) ? 6 : 0;
	const std::size_t dimB = constraint->getDimVelocity()+constraint->getDimWrench();
	std::size_t cI = 0;
	Eigen::MatrixXd reduced(dim,dimB);
	bool wrenchConstraints = false;
	Eigen::MatrixXd wrenchLHS;
	std::size_t iWrench = 0;
	if (dimB > 0) {
		for (std::size_t i = 0; i < 6; i++) {
			const Mode &modeI = constraintModes[i];
			if (modeI == Velocity) {
				std::size_t cJ = 0;
				for (std::size_t j = 0; j < 6; j++) {
					const Mode &modeJ = constraintModesB[j];
					if (modeJ == Velocity || modeJ == Wrench) {
						reduced(cI,cJ) = rotated(i,j);
						cJ++;
					}
				}
				/*
				for (std::size_t j = 0; j < 6; j++) {
					reduced(cI,j) = rotated(i,j);
				}
				 */
				cI++;
			} else if (modeI == Wrench) {
				if (!wrenchConstraints) {
					wrenchConstraints = true;
					wrenchLHS = getWrenchModelLHS(constraint);
					RW_ASSERT(wrenchLHS.cols() == 6);
					RW_ASSERT(wrenchLHS.rows() == (int)getDimWrench());
				}
				std::size_t cJ = 0;
				for (std::size_t j = 0; j < 6; j++) {
					const Mode &modeJ = constraintModesB[j];
					if (modeJ == Velocity || modeJ == Wrench) {
						reduced(cI,cJ) = wrenchLHS(iWrench,j);
						cJ++;
					}
				}
				/*
				for (std::size_t j = 0; j < 6; j++) {
					reduced(cI,j) = wrenchLHS(iWrench,j);
				}
				*/
				iWrench++;
				cI++;
			}
		}
	}
	return reduced;
}

Eigen::MatrixXd TNTConstraint::getWrenchModelLHS(const TNTConstraint* constraint) const {
	return Eigen::MatrixXd::Zero(getDimWrench(),6);
}

Eigen::VectorXd TNTConstraint::getWrenchModelRHS() const {
	return Eigen::VectorXd::Zero(getDimWrench());
}
