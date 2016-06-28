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

#include "ODEConstraint.hpp"
#include "ODEBody.hpp"
#include "ODESimulator.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rwsim/dynamics/Constraint.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;

ODEConstraint::ODEConstraint(rw::common::Ptr<const Constraint> constraint, const ODEBody* const parent, const ODEBody* const child, const ODESimulator* const simulator):
	_rwConstraint(constraint),
	_parent(parent),
	_child(child),
	_world(simulator->getODEWorldId()),
	_spring(NULL),
	_useSpringFrictionLin(false),
	_useSpringFrictionAng(false),
	_springFrictionLin(0.01),
	_springFrictionAng(0.01)
{
	createJoint();
	setLimits();
}

ODEConstraint::~ODEConstraint() {
	if (_spring != NULL)
		deleteSpring();
	if (_rwConstraint->getType() != Constraint::Free)
		destroyJoint();
}

void ODEConstraint::createJoint() {
	// Joint positions and directions are found in world coordinates
	const Transform3D<> wTparent = _parent->getTransform();
	const Transform3D<> parentTconstraint = _rwConstraint->getTransform();
	const Transform3D<> wTconstraint = wTparent*parentTconstraint;
	const Vector3D<> hpos = wTconstraint.P();
	const Vector3D<> xaxis = wTconstraint.R() * Vector3D<>::x();
	const Vector3D<> yaxis = wTconstraint.R() * Vector3D<>::y();
	const Vector3D<> zaxis = wTconstraint.R() * Vector3D<>::z();
	const Constraint::ConstraintType type = _rwConstraint->getType();
	const dBodyID first = _parent->getBodyID();
	const dBodyID second = _child->getBodyID();
	if (type == Constraint::Fixed) {
		_jointId = dJointCreateFixed(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetFixed(_jointId);
	} else if (type == Constraint::Prismatic) {
		_jointId = dJointCreateSlider(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetSliderAxis(_jointId, zaxis(0) , zaxis(1), zaxis(2));
	} else if (type == Constraint::Revolute) {
		_jointId = dJointCreateHinge(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetHingeAxis(_jointId, zaxis(0) , zaxis(1), zaxis(2));
		dJointSetHingeAnchor(_jointId, hpos(0), hpos(1), hpos(2));
	} else if (type == Constraint::Universal) {
		_jointId = dJointCreateUniversal(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetUniversalAxis1(_jointId, xaxis(0) , xaxis(1), xaxis(2));
		dJointSetUniversalAxis2(_jointId, yaxis(0) , yaxis(1), yaxis(2));
		dJointSetUniversalAnchor(_jointId, hpos(0), hpos(1), hpos(2));
	} else if (type == Constraint::Spherical) {
		_jointId = dJointCreateBall(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetBallAnchor(_jointId, hpos(0), hpos(1), hpos(2));
	} else if (type == Constraint::Piston) {
		_jointId = dJointCreatePiston(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetPistonAnchor(_jointId, hpos(0), hpos(1), hpos(2));
		dJointSetPistonAxis(_jointId, zaxis(0) , zaxis(1), zaxis(2));
	} else if (type == Constraint::PrismaticRotoid) {
		_jointId = dJointCreatePR(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetPRAnchor(_jointId, hpos(0), hpos(1), hpos(2));
		dJointSetPRAxis1(_jointId, zaxis(0) , zaxis(1), zaxis(2));
		dJointSetPRAxis2(_jointId, xaxis(0) , xaxis(1), xaxis(2));
	} else if (type == Constraint::PrismaticUniversal) {
		_jointId = dJointCreatePU(_world, 0);
		dJointAttach(_jointId, first, second);
		dJointSetPUAnchor(_jointId, hpos(0), hpos(1), hpos(2));
		dJointSetPUAxis1(_jointId, xaxis(0) , xaxis(1), xaxis(2));
		dJointSetPUAxis2(_jointId, yaxis(0) , yaxis(1), yaxis(2));
		dJointSetPUAxis3(_jointId, zaxis(0) , zaxis(1), zaxis(2));
	} else if (type == Constraint::Free) {
		// Do nothing
	} else {
		RW_THROW("Unsupported Constraint type!");
	}
}

void ODEConstraint::setLimits() const {
	const std::size_t dof = _rwConstraint->getDOF();
	for (std::size_t i = 0; i < dof; i++) {
		const Constraint::Limit limit = _rwConstraint->getLimit(i);
		if (limit.lowOn)
			setLoStop(i,limit.low);
		if (limit.highOn)
			setHiStop(i,limit.high);
	}
}

void ODEConstraint::destroyJoint() {
	dJointDestroy(_jointId);
}

void ODEConstraint::decomposeCompliance(const Eigen::MatrixXd &compliance, const Spring &spring, ComplianceDecomposition &dec, double tolerance) {
	// Do SVD on compliance matrix
	Eigen::MatrixXd U;
	Eigen::MatrixXd V;
	Eigen::VectorXd sigmaTmp;
	LinearAlgebra::svd(compliance,U,sigmaTmp,V);
	// Zero out close to singular directions
	const Eigen::VectorXd sigma = (sigmaTmp.array().abs() < tolerance).select(0, sigmaTmp);
	const Eigen::VectorXd sigmaInv = (sigma.array().abs() > 0).select(sigma.array().inverse(), 0);
	// Construct the Moore-Penrose pseudoinverse
	dec.cachedCompliance = compliance;
	dec.inverse = V*sigmaInv.asDiagonal()*U.adjoint(); // Moore-Penrose pseudoinverse

	// Construct vector of fixed directions (the singular directions), and the free directions.
	// Fixed directions must be either linear or angular, but free directions can be a combination.
	// Note that we are not in cartesian space here, but in a reduced space according to the DOF of the constraint.
	std::vector<std::vector<double> > linFixed;
	std::vector<std::vector<double> > angFixed;
	std::vector<std::vector<double> > free;
	for (Eigen::DenseIndex i = 0; i < sigma.rows(); i++) {
		Eigen::VectorXd vec;
		if (std::fabs((double)sigma(i)) < tolerance)
			vec = V.col(i);
		else
			vec = U.col(i);
		bool lin = false;
		bool ang = false;
		std::vector<double> linVec(spring.linComp,0);
		std::vector<double> angVec(spring.angComp,0);
		std::vector<double> freeVec(spring.linComp+spring.angComp,0);

		double linInfNorm = 0;
		double angInfNorm = 0;
		for (Eigen::DenseIndex j = 0; j < vec.rows(); j++) {
			if (j < spring.linComp) {
				linVec[j] = vec(j);
				if (vec(j) > linInfNorm)
					linInfNorm = vec(j);
			} else {
				angVec[j-spring.linComp] = vec(j);
				if (vec(j) > angInfNorm)
					angInfNorm = vec(j);
			}
			freeVec[j] = vec(j);
		}
		if (linInfNorm < 1e-12)
			lin = true;
		if (angInfNorm < 1e-12)
			ang = true;

		if (std::fabs((double)sigma(i)) < tolerance) {
			if (lin && ang)
				RW_THROW("Null space of compliance matrix spans vectors that are mixed linear and angular.");
			if (lin)
				linFixed.push_back(linVec);
			if (ang)
				angFixed.push_back(angVec);
		} else {
			free.push_back(freeVec);
		}

		if (linInfNorm > 0 && linInfNorm < 1e-12) {
			RW_WARN("ODEConstraint (decomposeCompliance): the null space of the compliance matrix gives a linear direction with a very small infinity norm (" << linInfNorm << ") - expected zero - the direction will be fixed anyway.");
		}
		if (angInfNorm > 0 && angInfNorm < 1e-12) {
			RW_WARN("ODEConstraint (decomposeCompliance): the null space of the compliance matrix gives a angular direction with a very small infinity norm (" << linInfNorm << ") - expected zero - the direction will be fixed anyway.");
		}
	}

	// Clear previous directions
	dec.linFixedDirs.clear();
	dec.angFixedDirs.clear();
	dec.freeDirs.clear();

	// Now the fixed linear directions are found in cartesian space.
	BOOST_FOREACH(const std::vector<double> &dir, linFixed) {
		Vector3D<> cDir;
		for (std::size_t i = 0; i < dir.size(); i++) {
			cDir += dir[i]*spring.rowToDir[i];
		}
		dec.linFixedDirs.push_back(cDir);
	}
	// Now the fixed angular directions are found in cartesian space.
	BOOST_FOREACH(const std::vector<double> &dir, angFixed) {
		Vector3D<> cDir;
		for (std::size_t i = 0; i < dir.size(); i++) {
			cDir += dir[i]*spring.rowToDir[i+spring.linComp];
		}
		dec.angFixedDirs.push_back(cDir);
	}
	// Now the free directions are found.
	BOOST_FOREACH(const std::vector<double> &dir, free) {
		VectorND<6> cDir;
		for (std::size_t i = 0; i < 6; i++)
			cDir[i] = 0;
		for (std::size_t i = 0; i < dir.size(); i++) {
			Vector3D<> d = dir[i]*spring.rowToDir[i];
			if (i < spring.linComp) {
				cDir[0] += d[0];
				cDir[1] += d[1];
				cDir[2] += d[2];
			} else {
				cDir[3] += d[0];
				cDir[4] += d[1];
				cDir[5] += d[2];
			}
		}
		dec.freeDirs.push_back(cDir);
	}
}

Eigen::VectorXd ODEConstraint::getX(const Transform3D<> &wTchild, const Transform3D<> &wTconstraint, const State &state) const {
	const Transform3D<> pTc = inverse(wTconstraint)*wTchild;
	const VelocityScrew6D<> pos(pTc);
	Eigen::VectorXd Xcur(_spring->linComp+_spring->angComp);
	unsigned int redI = 0;
	for(std::size_t i = 0; i < 6; i++) {
		if (_spring->comp[i]) {
			Xcur[redI] = pos[i];
			redI++;
		}
	}
	return Xcur;
}

Eigen::VectorXd ODEConstraint::getXd(const Transform3D<> &wTparent, const Transform3D<> &wTconstraint, const State &state) const {
	const Vector3D<> linVelP = (_parent->getType() == ODEBody::FIXED)? Vector3D<>::zero() : ODEUtil::toVector3D(dBodyGetLinearVel(_parent->getBodyID()));
	const Vector3D<> linVelC = (_child->getType() == ODEBody::FIXED)? Vector3D<>::zero() : ODEUtil::toVector3D(dBodyGetLinearVel(_child->getBodyID()));
	const Vector3D<> angVelP = (_parent->getType() == ODEBody::FIXED)? Vector3D<>::zero() : ODEUtil::toVector3D(dBodyGetAngularVel(_parent->getBodyID()));
	const Vector3D<> angVelC = (_child->getType() == ODEBody::FIXED)? Vector3D<>::zero() : ODEUtil::toVector3D(dBodyGetAngularVel(_child->getBodyID()));
	const Vector3D<> angVelCon = angVelP+cross(angVelP,wTconstraint.P()-wTparent.P());
	const VelocityScrew6D<> relVelP(linVelC-linVelP, EAA<>(angVelC-angVelCon));
	const VelocityScrew6D<> relVelCon(inverse(wTconstraint.R())*relVelP);
	Eigen::VectorXd XcurD(_spring->linComp+_spring->angComp);
	unsigned int redI = 0;
	for(std::size_t i = 0; i < 6; i++) {
		if (_spring->comp[i]) {
			XcurD[redI] = relVelCon[i];
			redI++;
		}
	}
	return XcurD;
}

std::pair<Vector3D<>, Vector3D<> > ODEConstraint::toCartesian(const Eigen::VectorXd &reduced) const {
	std::pair<Vector3D<>, Vector3D<> > pair;
	Vector3D<> lin;
	Vector3D<> ang;
	for (std::size_t i = 0; i < _spring->linComp; i++)
		lin += reduced[i]*_spring->rowToDir[i];
	for (std::size_t i = 0; i < _spring->angComp; i++)
		ang += reduced[i+_spring->linComp]*_spring->rowToDir[i+_spring->linComp];
	pair.first = lin;
	pair.second = ang;
	return pair;
}

void ODEConstraint::setMotorDirLin(const rw::math::Vector3D<> axis, unsigned int dirNumber) const {
	dJointSetLMotorAxis(_spring->motorLin, dirNumber, 1, axis(0) , axis(1), axis(2));
	dJointSetLMotorParam(_spring->motorLin,dParamFMax+dParamGroup*dirNumber,_springFrictionLin);
	dJointSetLMotorParam(_spring->motorLin,dParamVel+dParamGroup*dirNumber,0);
}

void ODEConstraint::setMotorDirAng(const rw::math::Vector3D<> axis, unsigned int dirNumber) const {
	dJointSetAMotorAxis(_spring->motorAng, dirNumber, 1, axis(0) , axis(1), axis(2));
	dJointSetAMotorParam(_spring->motorAng,dParamFMax+dParamGroup*dirNumber,_springFrictionAng);
	dJointSetAMotorParam(_spring->motorAng,dParamVel+dParamGroup*dirNumber,0);
}

void ODEConstraint::update(const Simulator::UpdateInfo& dt, State& state) {
	const Constraint::SpringParams params = _rwConstraint->getSpringParams();

	// If spring is enabled/disabled or compliance is changed, this is handled here.
	bool newCompliance = false;
	if (_spring != NULL && !params.enabled) {
		deleteSpring();
		return;
	} else if (_spring == NULL && params.enabled) {
		createSpring();
		decomposeCompliance(params.compliance,*_spring,_spring->cDec);
		newCompliance = true;
	}
	if (!params.enabled)
		return;

	// Update the compliance if it has been changed
	const Eigen::MatrixXd C = params.compliance;
	if (C.cwiseNotEqual(_spring->cDec.cachedCompliance).count() > 0) {
		decomposeCompliance(params.compliance,*_spring,_spring->cDec);
		newCompliance = true;
	}

	const Frame* parentFrame = _rwConstraint->getBody1()->getBodyFrame();
	const Frame* childFrame = _rwConstraint->getBody2()->getBodyFrame();
	const Transform3D<> wTparent = Kinematics::worldTframe(parentFrame,state);
	const Transform3D<> parentTconstraint = _rwConstraint->getTransform();
	const Transform3D<> wTconstraint = wTparent*parentTconstraint;
	const Transform3D<> wTchild = Kinematics::worldTframe(childFrame,state);

	// Find the current relative position and velocity in reduced space.
	const Eigen::VectorXd Xcur = getX(wTchild, wTconstraint, state);
	const Eigen::VectorXd XcurD = getXd(wTparent, wTconstraint, state);

	// Find the same values in full cartesian space.
	std::pair<Vector3D<>, Vector3D<> > pos = toCartesian(Xcur);

	// Fixate the linear singular directions
	std::vector<Vector3D<> > motorLinDirs;
	for (unsigned int i = 0; i < _spring->cDec.linFixedDirs.size(); i++) {
		const Vector3D<> axis = wTconstraint.R()*_spring->cDec.linFixedDirs[i];
		motorLinDirs.push_back(axis);
		if (newCompliance) {
			dJointSetLMotorAxis(_spring->motorLin, i, 1, axis(0) , axis(1), axis(2));
			dJointSetLMotorParam(_spring->motorLin,dParamFMax+dParamGroup*i,1000);
		}
		const Vector3D<> axisCon = _spring->cDec.linFixedDirs[i];
		dJointSetLMotorParam(_spring->motorLin,dParamVel+dParamGroup*i,(1./dt.dt)*dot(axisCon,pos.first));
	}
	// Fixate the angular singular directions
	std::vector<Vector3D<> > motorAngDirs;
	for (unsigned int i = 0; i < _spring->cDec.angFixedDirs.size(); i++) {
		const Vector3D<> axis = wTconstraint.R()*_spring->cDec.angFixedDirs[i];
		motorAngDirs.push_back(axis);
		if (newCompliance) {
			dJointSetAMotorAxis(_spring->motorAng, i, 1, axis(0) , axis(1), axis(2));
			dJointSetAMotorParam(_spring->motorAng,dParamFMax+dParamGroup*i,1000);
		}
		const Vector3D<> axisCon = _spring->cDec.angFixedDirs[i];
		dJointSetAMotorParam(_spring->motorAng,dParamVel+dParamGroup*i,(1./dt.dt)*dot(axisCon,pos.second));
	}

	// Find the force and torque to apply in cartesian space.
	const Eigen::MatrixXd& K = _spring->cDec.inverse;
	const Eigen::VectorXd ft = K*Xcur;
	std::pair<Vector3D<>, Vector3D<> > cart = toCartesian(ft);
	{
		const Eigen::VectorXd ftD = params.damping*XcurD;
		std::pair<Vector3D<>, Vector3D<> > damping = toCartesian(ftD);
		// Now remove the damping in the directions that has been fixed.
		for (unsigned int i = 0; i < _spring->cDec.linFixedDirs.size(); i++) {
			const Vector3D<> axis = _spring->cDec.linFixedDirs[i];
			damping.first = damping.first-dot(axis,damping.first)*axis;
		}
		for (unsigned int i = 0; i < _spring->cDec.angFixedDirs.size(); i++) {
			const Vector3D<> axis = _spring->cDec.angFixedDirs[i];
			damping.second = damping.second-dot(axis,damping.second)*axis;
		}
		cart.first = cart.first + damping.first;
		cart.second = cart.second + damping.second;
	}

	// Now add the force to the bodies
	const Transform3D<> childTconstraint = inverse(wTchild)*wTconstraint;
	const Vector3D<> cartForceP = parentTconstraint.R()*cart.first;
	const Vector3D<> cartTorqueP = parentTconstraint.R()*cart.second;
	const Vector3D<> cartForceC = childTconstraint.R()*cart.first;
	const Vector3D<> cartTorqueC = childTconstraint.R()*cart.second;
	_rwConstraint->getBody1()->addForceToPos(cartForceP,parentTconstraint.P(),state);
	_rwConstraint->getBody2()->addForceToPos(-cartForceC,childTconstraint.P(),state);
	_rwConstraint->getBody1()->addTorque(cartTorqueP,state);
	_rwConstraint->getBody2()->addTorque(-cartTorqueC,state);
	// In the case where the body frame is not in the com we add a torque to the body to account for this
	const Vector3D<> com1 = _rwConstraint->getBody1()->getInfo().masscenter;
	const Vector3D<> com2 = _rwConstraint->getBody2()->getInfo().masscenter;
	_rwConstraint->getBody1()->addTorque(cross(parentTconstraint.P()-com1,cartForceP),state);
	_rwConstraint->getBody2()->addTorque(cross(childTconstraint.P()-com2,-cartForceC),state);

	// Motors are now added for static friction (if enabled).

	// First friction is applied in the main direction of the applied force (if possible)
	if (_useSpringFrictionLin) {
		if ((_spring->linComp-motorLinDirs.size()) > 0) {
			const Vector3D<> axis = normalize(wTconstraint.R()*cart.first);
			if (!(axis == Vector3D<>::zero())) {
				setMotorDirLin(axis, motorLinDirs.size());
				motorLinDirs.push_back(axis);
			} else {
				if (motorLinDirs.size() == 0) {
					setMotorDirLin(wTconstraint.R()*_spring->rowToDir[0], motorLinDirs.size());
					motorLinDirs.push_back(wTconstraint.R()*_spring->rowToDir[0]);
				} else if (motorLinDirs.size() == 1 && _spring->angComp == 2) {
					const Vector3D<> dir1 = _spring->rowToDir[0];
					const Vector3D<> dir2 = _spring->rowToDir[1];
					const Vector3D<> rot = normalize(cross(dir1,dir2));
					const Vector3D<> axis = normalize(cross(rot,motorLinDirs[0]));
					setMotorDirLin(axis, motorLinDirs.size());
					motorLinDirs.push_back(axis);
				} else if (motorLinDirs.size() == 1 && _spring->angComp == 3) {
					const Rotation3D<> dirs = EAA<>(wTconstraint.R()*Vector3D<>::z(),motorLinDirs[0]).toRotation3D();
					setMotorDirLin(dirs.getCol(0), motorLinDirs.size());
					motorLinDirs.push_back(dirs.getCol(0));
					setMotorDirLin(dirs.getCol(1), motorLinDirs.size());
					motorLinDirs.push_back(dirs.getCol(1));
				} else if (motorLinDirs.size() == 2) {
					const Vector3D<> axis = normalize(cross(motorLinDirs[0],motorLinDirs[1]));
					setMotorDirLin(axis, motorLinDirs.size());
					motorLinDirs.push_back(axis);
				}
			}
		}

		// If there are more free linear directions, these are added now
		if (_spring->linComp == 3 && motorLinDirs.size() == 2) {
			const Vector3D<> axis = normalize(cross(motorLinDirs[0],motorLinDirs[1]));
			setMotorDirLin(axis, motorLinDirs.size());
			motorLinDirs.push_back(axis);
		} else if (_spring->linComp == 3 && motorLinDirs.size() == 1) {
			Rotation3D<> dirs = EAA<>(wTconstraint.R()*Vector3D<>::z(),motorLinDirs[0]).toRotation3D();
			setMotorDirLin(dirs.getCol(0), motorLinDirs.size());
			motorLinDirs.push_back(dirs.getCol(0));
			setMotorDirLin(dirs.getCol(1), motorLinDirs.size());
			motorLinDirs.push_back(dirs.getCol(1));
		} else if (_spring->linComp == 2 && motorLinDirs.size() == 1) {
			const Vector3D<> dir1 = _spring->rowToDir[0];
			const Vector3D<> dir2 = _spring->rowToDir[1];
			const Vector3D<> rot = normalize(cross(dir1,dir2));
			const Vector3D<> axis = normalize(cross(wTconstraint.R()*rot,motorLinDirs[0]));
			setMotorDirLin(axis, motorLinDirs.size());
			motorLinDirs.push_back(axis);
		}
	}

	// First angular friction is applied in the main direction of the applied torque (if possible)
	if (_useSpringFrictionAng) {
		if ((_spring->angComp-motorAngDirs.size()) > 0) {
			const Vector3D<> axis = normalize(wTconstraint.R()*cart.second);
			if (!(axis == Vector3D<>::zero())) {
				setMotorDirAng(axis, motorAngDirs.size());
				motorAngDirs.push_back(axis);
			} else {
				if (motorAngDirs.size() == 0) {
					setMotorDirAng(wTconstraint.R()*_spring->rowToDir[_spring->linComp], motorAngDirs.size());
					motorAngDirs.push_back(wTconstraint.R()*_spring->rowToDir[_spring->linComp]);
				} else if (motorAngDirs.size() == 1 && _spring->angComp == 2) {
					const Vector3D<> dir1 = _spring->rowToDir[0+_spring->linComp];
					const Vector3D<> dir2 = _spring->rowToDir[1+_spring->linComp];
					const Vector3D<> rot = normalize(cross(dir1,dir2));
					const Vector3D<> axis = normalize(cross(rot,motorAngDirs[0]));
					setMotorDirAng(axis, motorAngDirs.size());
					motorAngDirs.push_back(axis);
				} else if (motorAngDirs.size() == 1 && _spring->angComp == 3) {
					const Rotation3D<> dirs = EAA<>(wTconstraint.R()*Vector3D<>::z(),motorAngDirs[0]).toRotation3D();
					setMotorDirAng(dirs.getCol(0), motorAngDirs.size());
					motorAngDirs.push_back(dirs.getCol(0));
					setMotorDirAng(dirs.getCol(1), motorAngDirs.size());
					motorAngDirs.push_back(dirs.getCol(1));
				} else if (motorAngDirs.size() == 2) {
					const Vector3D<> axis = normalize(cross(motorAngDirs[0],motorAngDirs[1]));
					setMotorDirAng(axis, motorAngDirs.size());
					motorAngDirs.push_back(axis);
				}
			}
		}

		// If there are more free angular directions, these are added now
		if (_spring->angComp == 3 && motorAngDirs.size() == 2) {
			const Vector3D<> axis = normalize(cross(motorAngDirs[0],motorAngDirs[1]));
			setMotorDirAng(axis, motorAngDirs.size());
			motorAngDirs.push_back(axis);
		} else if (_spring->angComp == 3 && motorAngDirs.size() == 1) {
			const Rotation3D<> dirs = EAA<>(wTconstraint.R()*Vector3D<>::z(),motorAngDirs[0]).toRotation3D();
			setMotorDirAng(dirs.getCol(0), motorAngDirs.size());
			motorAngDirs.push_back(dirs.getCol(0));
			setMotorDirAng(dirs.getCol(1), motorAngDirs.size());
			motorAngDirs.push_back(dirs.getCol(1));
		} else if (_spring->angComp == 2 && motorAngDirs.size() == 1) {
			const Vector3D<> dir1 = _spring->rowToDir[0+_spring->linComp];
			const Vector3D<> dir2 = _spring->rowToDir[1+_spring->linComp];
			const Vector3D<> rot = normalize(cross(dir1,dir2));
			const Vector3D<> axis = normalize(cross(wTconstraint.R()*rot,motorAngDirs[0]));
			setMotorDirAng(axis, motorAngDirs.size());
			motorAngDirs.push_back(axis);
		}
	}
}

void ODEConstraint::createSpring() {
	const Constraint::ConstraintType type = _rwConstraint->getType();
	_spring = new Spring();
	_spring->linComp = _rwConstraint->getDOFLinear();
	_spring->angComp = _rwConstraint->getDOFAngular();
	// Construct map of compliant directions based on constraint type.
	for (std::size_t i = 0; i < 6; i++)
		_spring->comp[i] = false;
	if (_spring->linComp > 0) {
		if (type == Constraint::Free) {
			_spring->comp[0] = true;
			_spring->comp[1] = true;
			_spring->comp[2] = true;
			_spring->rowToDir.push_back(Vector3D<>::x());
			_spring->rowToDir.push_back(Vector3D<>::y());
			_spring->rowToDir.push_back(Vector3D<>::z());
		} else {
			_spring->comp[2] = true;
			_spring->rowToDir.push_back(Vector3D<>::z());
		}
	}
	if (_spring->angComp > 0) {
		if (type == Constraint::Revolute || type == Constraint::Piston) {
			_spring->comp[5] = true;
			_spring->rowToDir.push_back(Vector3D<>::z());
		} else if (type == Constraint::Universal || type == Constraint::PrismaticUniversal) {
			_spring->comp[3] = true;
			_spring->comp[4] = true;
			_spring->rowToDir.push_back(Vector3D<>::x());
			_spring->rowToDir.push_back(Vector3D<>::y());
		} else if (type == Constraint::Spherical || type == Constraint::Free) {
			_spring->comp[3] = true;
			_spring->comp[4] = true;
			_spring->comp[5] = true;
			_spring->rowToDir.push_back(Vector3D<>::x());
			_spring->rowToDir.push_back(Vector3D<>::y());
			_spring->rowToDir.push_back(Vector3D<>::z());
		} else if (type == Constraint::PrismaticRotoid) {
			_spring->comp[3] = true;
			_spring->rowToDir.push_back(Vector3D<>::x());
		}
	}
	// Construct linear motor if there are linear DOFs
	if (_useSpringFrictionLin && _spring->linComp > 0) {
		_spring->motorLin = dJointCreateLMotor(_world, 0);
		dJointAttach(_spring->motorLin, _parent->getBodyID(), _child->getBodyID());
		dJointSetLMotorNumAxes(_spring->motorLin, _spring->linComp);
		for (unsigned int i = 0; i < _spring->linComp; i++) {
			dJointSetLMotorParam(_spring->motorLin,dParamFMax+dParamGroup*i,0);
			dJointSetLMotorParam(_spring->motorLin,dParamVel+dParamGroup*i,0);
		}
	}
	// Construct angular motor if there are angular DOFs
	if (_useSpringFrictionAng && _spring->angComp > 0) {
		_spring->motorAng = dJointCreateAMotor(_world, 0);
		dJointAttach(_spring->motorAng, _parent->getBodyID(), _child->getBodyID());
		dJointSetAMotorNumAxes(_spring->motorAng, _spring->angComp);
		for (unsigned int i = 0; i < _spring->angComp; i++) {
			dJointSetAMotorParam(_spring->motorAng,dParamFMax+dParamGroup*i,0);
			dJointSetAMotorParam(_spring->motorAng,dParamVel+dParamGroup*i,0);
		}
	}
}

void ODEConstraint::deleteSpring() {
	if (_useSpringFrictionLin && _spring->linComp > 0)
		dJointDestroy (_spring->motorLin);
	if (_useSpringFrictionAng && _spring->angComp > 0)
		dJointDestroy (_spring->motorAng);
	delete _spring;
	_spring = NULL;
}

void ODEConstraint::setLoStop(std::size_t dof, double limit) const {
	const Constraint::ConstraintType type = _rwConstraint->getType();
	RW_ASSERT(dof == 0 || dof == 1);
	if (type == Constraint::Fixed) {
	} else if (type == Constraint::Prismatic && dof == 0) {
		dJointSetSliderParam(_jointId,dParamLoStop, limit);
	} else if (type == Constraint::Revolute && dof == 0) {
		dJointSetHingeParam(_jointId,dParamLoStop, limit);
	} else if (type == Constraint::Universal) {
		if (dof == 0)
			dJointSetUniversalParam(_jointId,dParamLoStop, limit);
		else if (dof == 1)
			dJointSetUniversalParam(_jointId,dParamLoStop2, limit);
	} else if (type == Constraint::Spherical) {
		RW_THROW("Spherical constraint is unsupported for lower limit");
	} else if (type == Constraint::Piston && dof == 0) {
		dJointSetPistonParam(_jointId,dParamLoStop, limit);
	} else if (type == Constraint::PrismaticRotoid) {
		if (dof == 0)
			dJointSetPRParam (_jointId,dParamLoStop,limit);
		else if (dof == 1)
			dJointSetPRParam (_jointId,dParamLoStop2,limit);
	} else if (type == Constraint::PrismaticUniversal) {
		if (dof == 0)
			dJointSetPUParam(_jointId,dParamLoStop,limit);
		else if (dof == 1)
			dJointSetPUParam(_jointId,dParamLoStop2,limit);
	} else if (type == Constraint::Free) {
		// Do nothing
	} else {
		RW_THROW("Constraint is unsupported for lower limit");
	}
}
void ODEConstraint::setHiStop(std::size_t dof, double limit) const {
	const Constraint::ConstraintType type = _rwConstraint->getType();
	RW_ASSERT(dof == 0 || dof == 1);
	if (type == Constraint::Fixed) {
	} else if (type == Constraint::Prismatic && dof == 0) {
		dJointSetSliderParam(_jointId,dParamHiStop, limit);
	} else if (type == Constraint::Revolute && dof == 0) {
		dJointSetHingeParam(_jointId,dParamHiStop, limit);
	} else if (type == Constraint::Universal) {
		if (dof == 0)
			dJointSetUniversalParam(_jointId,dParamHiStop, limit);
		else if (dof == 1)
			dJointSetUniversalParam(_jointId,dParamHiStop2, limit);
	} else if (type == Constraint::Spherical) {
		RW_THROW("Spherical constraint is unsupported for higher limit");
	} else if (type == Constraint::Piston && dof == 0) {
		dJointSetPistonParam(_jointId,dParamHiStop, limit);
	} else if (type == Constraint::PrismaticRotoid) {
		if (dof == 0)
			dJointSetPRParam (_jointId,dParamHiStop,limit);
		else if (dof == 1)
			dJointSetPRParam (_jointId,dParamHiStop2,limit);
	} else if (type == Constraint::PrismaticUniversal) {
		if (dof == 0)
			dJointSetPUParam(_jointId,dParamHiStop,limit);
		else if (dof == 1)
			dJointSetPUParam(_jointId,dParamHiStop2,limit);
	} else if (type == Constraint::Free) {
		// Do nothing
	} else {
		RW_THROW("Constraint is unsupported for higher limit");
	}
}
