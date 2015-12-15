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

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include "RWPEBody.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyFixed.hpp"
#include "RWPEBodyKinematic.hpp"
#include "RWPEConstraint.hpp"
#include "RWPEIntegrator.hpp"
#include "RWPEIslandState.hpp"

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

RWPEConstraint::RWPEConstraint(const RWPEBody* parent, const RWPEBody* child):
	_parent(parent),
	_child(child)
{
};

RWPEConstraint::~RWPEConstraint() {
}

const RWPEBody* RWPEConstraint::getParent() const {
	return _parent;
}

const RWPEBody* RWPEConstraint::getChild() const {
	return _child;
}

const Vector3D<>& RWPEConstraint::getPositionParent() const {
	return _posParent;
}

const Vector3D<>& RWPEConstraint::getPositionChild() const {
	return _posChild;
}

const Rotation3D<>& RWPEConstraint::getLinearRotationParent() const {
	return _rotLinParent;
}

const Rotation3D<>& RWPEConstraint::getLinearRotationChild() const {
	return _rotLinChild;
}

const Rotation3D<>& RWPEConstraint::getAngularRotationParent() const {
	return _rotAngParent;
}

const Rotation3D<>& RWPEConstraint::getAngularRotationChild() const {
	return _rotAngChild;
}

Vector3D<> RWPEConstraint::getPositionParentW(const RWPEIslandState &state) const {
	return _parent->getWorldTcom(state)*_posParent;
}

Vector3D<> RWPEConstraint::getPositionChildW(const RWPEIslandState &state) const {
	return _child->getWorldTcom(state)*_posChild;
}

Rotation3D<> RWPEConstraint::getLinearRotationParentW(const RWPEIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotLinParent;
}

Rotation3D<> RWPEConstraint::getLinearRotationParentForceW(const RWPEIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotLinParent;
}

Rotation3D<> RWPEConstraint::getLinearRotationChildW(const RWPEIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotLinChild;
}

Rotation3D<> RWPEConstraint::getLinearRotationChildForceW(const RWPEIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotLinChild;
}

Rotation3D<> RWPEConstraint::getAngularRotationParentW(const RWPEIslandState &state) const {
	return _parent->getWorldTcom(state).R()*_rotAngParent;
}

Rotation3D<> RWPEConstraint::getAngularRotationChildW(const RWPEIslandState &state) const {
	return _child->getWorldTcom(state).R()*_rotAngChild;
}

VelocityScrew6D<> RWPEConstraint::getVelocityParentW(const RWPEIslandState &islandState, const State& rwstate) const {
	const VelocityScrew6D<> vel = _parent->getVelocityW(rwstate,islandState);
	const Vector3D<> R = _parent->getWorldTcom(islandState).P();
	const Vector3D<> pos = getPositionParentW(islandState);
	const Vector3D<> lin = vel.linear() + cross(vel.angular().angle()*vel.angular().axis(),pos-R);
	return VelocityScrew6D<>(lin,vel.angular());
}

VelocityScrew6D<> RWPEConstraint::getVelocityChildW(const RWPEIslandState &islandState, const State& rwstate) const {
	const VelocityScrew6D<> vel = _child->getVelocityW(rwstate,islandState);
	const Vector3D<> R = _child->getWorldTcom(islandState).P();
	const Vector3D<> pos = getPositionChildW(islandState);
	const Vector3D<> lin = vel.linear() + cross(vel.angular().angle()*vel.angular().axis(),pos-R);
	return VelocityScrew6D<>(lin,vel.angular());
}

Wrench6D<> RWPEConstraint::getWrench(const RWPEIslandState &islandState) const {
	return islandState.getWrench(this);
}

Wrench6D<> RWPEConstraint::getWrenchApplied(const RWPEIslandState &islandState) const {
	return islandState.getWrenchApplied(this);
}

Wrench6D<> RWPEConstraint::getWrenchConstraint(const RWPEIslandState &islandState) const {
	return islandState.getWrenchConstraint(this);
}

void RWPEConstraint::clearWrench(RWPEIslandState &islandState) {
	clearWrenchApplied(islandState);
	clearWrenchConstraint(islandState);
}

void RWPEConstraint::clearWrenchApplied(RWPEIslandState &islandState) {
	islandState.setWrenchApplied(this,Wrench6D<>());
}

void RWPEConstraint::clearWrenchConstraint(RWPEIslandState &islandState) {
	islandState.setWrenchConstraint(this,Wrench6D<>());
}

void RWPEConstraint::applyWrench(RWPEIslandState &islandState, const Wrench6D<>& wrench) {
	const Wrench6D<> cur = getWrenchApplied(islandState);
	islandState.setWrenchApplied(this,cur+wrench);
}

Eigen::VectorXd RWPEConstraint::getRHS(double h, const Vector3D<> &gravity, bool discontinuity, const std::list<RWPEConstraint*>& constraints0, const std::list<RWPEConstraint*>& constraintsH, const State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH) const {
	const std::size_t dim = 6-getDimFree();
	RW_ASSERT(dim >= 0);
	Eigen::VectorXd res = Eigen::VectorXd::Zero(dim);
	Eigen::Matrix<double, 6, 1> velParent = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> velChild = Eigen::Matrix<double, 6, 1>::Zero();
	if (const RWPEBodyFixed* const body = dynamic_cast<const RWPEBodyFixed*>(_parent)) {
		// Do nothing
	} else if (const RWPEBodyKinematic* const body = dynamic_cast<const RWPEBodyKinematic*>(_parent)) {
		const Vector3D<>& R = body->getWorldTcom(islandStateH).P();
		const Vector3D<> pos = getPositionParentW(islandStateH);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,islandState0);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velParent << lin.e(), ang.e();
	} else if (const RWPEBodyDynamic* const body = dynamic_cast<const RWPEBodyDynamic*>(_parent)) {
		const RWPEBodyDynamic::RigidConfiguration* const config0 = body->getConfiguration(islandState0);
		const RWPEBodyDynamic::RigidConfiguration* const configH = body->getConfiguration(islandStateH);
		const Vector3D<> pos = getPositionParentW(islandStateH);
		const Wrench6D<> Wtot0 = body->getNetWrench(gravity,constraints0,islandState0,rwstate);
		const Wrench6D<> WextH = body->getExternalWrench(gravity,constraintsH,islandStateH,rwstate);
		const RWPEIntegrator* integrator = body->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		velParent = integrator->eqPointVelIndependent(pos,h,*config0,*configH,Wtot0.force(),Wtot0.torque(),WextH.force(),WextH.torque());
	} else {
		RW_THROW("RWPEConstraint (getRHS): the type of body \"" << _parent->get()->getName() << "\" is not supported!");
	}
	if (const RWPEBodyFixed* const body = dynamic_cast<const RWPEBodyFixed*>(_child)) {
		// Do nothing
	} else if (const RWPEBodyKinematic* const body = dynamic_cast<const RWPEBodyKinematic*>(_child)) {
		const Vector3D<>& R = body->getWorldTcom(islandStateH).P();
		const Vector3D<> pos = getPositionChildW(islandStateH);
		const VelocityScrew6D<> vel = body->getVelocityW(rwstate,islandState0);
		const Vector3D<> ang = vel.angular().angle()*vel.angular().axis();
		const Vector3D<> lin = vel.linear()+cross(ang,pos-R);
		velChild << lin.e(), ang.e();
	} else if (const RWPEBodyDynamic* const body = dynamic_cast<const RWPEBodyDynamic*>(_child)) {
		const RWPEBodyDynamic::RigidConfiguration* const config0 = body->getConfiguration(islandState0);
		const RWPEBodyDynamic::RigidConfiguration* const configH = body->getConfiguration(islandStateH);
		const Vector3D<> pos = getPositionChildW(islandStateH);
		const Wrench6D<> Wtot0 = body->getNetWrench(gravity,constraints0,islandState0,rwstate);
		const Wrench6D<> WextH = body->getExternalWrench(gravity,constraintsH,islandStateH,rwstate);
		const RWPEIntegrator* integrator = body->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		velChild = integrator->eqPointVelIndependent(pos,h,*config0,*configH,Wtot0.force(),Wtot0.torque(),WextH.force(),WextH.torque());
	} else {
		RW_THROW("RWPEConstraint (getRHS): the type of body \"" << _child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 1> total = velChild-velParent;

	// Rotate to local constraint coordinates:
	const Rotation3D<> Rlin = getLinearRotationParentW(islandStateH);
	const Rotation3D<> Rang = getAngularRotationParentW(islandStateH);
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

Eigen::MatrixXd RWPEConstraint::getLHS(const RWPEConstraint* constraint, double h, bool discontinuity, const State &rwstate, const RWPEIslandState &islandState) const {
	const RWPEBody* parent = _parent;
	const RWPEBody* child = _child;
	const RWPEBody* constraintParent = constraint->getParent();
	const RWPEBody* constraintChild = constraint->getChild();
	const std::size_t dim = getDimVelocity()+getDimWrench();
	Eigen::Matrix<double, 6, 6> matrixParent = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 6> matrixChild = Eigen::Matrix<double, 6, 6>::Zero();
	if (const RWPEBodyFixed* const body = dynamic_cast<const RWPEBodyFixed*>(parent)) {
		// Do nothing
	} else if (const RWPEBodyKinematic* const body = dynamic_cast<const RWPEBodyKinematic*>(parent)) {
		// Do nothing
	} else if (const RWPEBodyDynamic* const body = dynamic_cast<const RWPEBodyDynamic*>(parent)) {
		const RWPEBodyDynamic::RigidConfiguration* const config = body->getConfiguration(islandState);
		const Vector3D<> pos = getPositionParentW(islandState);
		if (parent == constraintParent) {
			const Vector3D<> constraintPos = constraint->getPositionParentW(islandState);
			const RWPEIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixParent = integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (parent == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(islandState);
			const RWPEIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixParent = -integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		}
	} else {
		RW_THROW("RWPEConstraint (getLHS): one of the bodies \"" << _parent->get()->getName() << "\" and \"" << _child->get()->getName() << "\" must be a RWPEBodyDynamic!");
	}
	if (const RWPEBodyFixed* const body = dynamic_cast<const RWPEBodyFixed*>(child)) {
		// Do nothing
	} else if (const RWPEBodyKinematic* const body = dynamic_cast<const RWPEBodyKinematic*>(child)) {
		// Do nothing
	} else if (const RWPEBodyDynamic* const body = dynamic_cast<const RWPEBodyDynamic*>(child)) {
		const RWPEBodyDynamic::RigidConfiguration* const config = body->getConfiguration(islandState);
		const Vector3D<> pos = getPositionChildW(islandState);
		if (child == constraintParent) {
			const Vector3D<> constraintPos = constraint->getPositionParentW(islandState);
			const RWPEIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixChild = -integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		} else if (child == constraintChild) {
			const Vector3D<> constraintPos = constraint->getPositionChildW(islandState);
			const RWPEIntegrator* integrator = body->getIntegrator();
			if (discontinuity)
				integrator = integrator->getDiscontinuityIntegrator();
			matrixChild = integrator->eqPointVelConstraintWrenchFactor(pos,h,constraintPos,*config,*config);
		}
	} else {
		RW_THROW("RWPEConstraint (getLHS): the type of body \"" << child->get()->getName() << "\" is not supported!");
	}
	const Eigen::Matrix<double, 6, 6> total = matrixParent+matrixChild;

	// Now we rotate the constraints and change to force and torque in local coordinates
	const Rotation3D<> Rlin = getLinearRotationParentW(islandState);
	const Rotation3D<> Rang = getAngularRotationParentW(islandState);
	const Rotation3D<> RlinB = constraint->getLinearRotationParentForceW(islandState);
	const Rotation3D<> RangB = constraint->getAngularRotationParentW(islandState);
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

Eigen::MatrixXd RWPEConstraint::getWrenchModelLHS(const RWPEConstraint* constraint) const {
	return Eigen::MatrixXd::Zero(getDimWrench(),6);
}

Eigen::VectorXd RWPEConstraint::getWrenchModelRHS() const {
	return Eigen::VectorXd::Zero(getDimWrench());
}
