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
#include <rwsim/dynamics/Constraint.hpp>
#include "RWPEBody.hpp"
#include "RWPEConstraintGeneric.hpp"

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

#define RWPE_SPRING_EIGENVALUE_SQRT_THRESHOLD 1e-6

struct RWPERWConstraint::ComplianceDecomposition {
	Eigen::MatrixXd cachedCompliance;
	Eigen::MatrixXd inverse;
	std::vector<Vector3D<> > linFixedDirs;
	std::vector<Vector3D<> > angFixedDirs;
	std::vector<VectorND<6> > freeDirs;
	std::vector<Mode> modes;
};

struct RWPERWConstraint::Spring {
	// Constraint based settings (static)
	std::vector<rw::math::Vector3D<> > rowToDir;
	bool comp[6];
	unsigned int linComp;
	unsigned int angComp;
	// Settings updated when compliance changes
	ComplianceDecomposition cDec;
};

RWPERWConstraint::RWPERWConstraint(rw::common::Ptr<const Constraint> constraint, const RWPEBody* parent, const RWPEBody* child):
	RWPEConstraint(parent,child),
	_rwConstraint(constraint),
	_modes(getModes(constraint)),
	_dimVel(6-constraint->getDOF()),
	_dimFree(constraint->getDOF()),
	_spring(NULL)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
}

RWPERWConstraint::~RWPERWConstraint() {
	if (_spring != NULL)
		delete _spring;
	_spring = NULL;
}

rw::common::Ptr<const Constraint> RWPERWConstraint::getConstraint() const {
	return _rwConstraint;
}

void RWPERWConstraint::update(RWPEIslandState &islandState, const State& rwstate) {
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

	if (newCompliance) {
		// If some directions should be fixed, the constraint is now modified
		Rotation3D<> pRcLin = _pTc.R();
		Rotation3D<> pRcAng = _pTc.R();
		Rotation3D<> cRcLin = _cTc.R();
		Rotation3D<> cRcAng = _cTc.R();
		_spring->cDec.modes = _modes;

		if (_spring->linComp > 0 && _spring->cDec.linFixedDirs.size() > 0) {
			bool changed = false;
			if (_spring->linComp == _spring->cDec.linFixedDirs.size()) {
				_spring->cDec.modes[0] = RWPEConstraint::Velocity;
				_spring->cDec.modes[1] = RWPEConstraint::Velocity;
				_spring->cDec.modes[2] = RWPEConstraint::Velocity;
			} else if (_spring->linComp == 2) {
				const Vector3D<> fixed = _spring->cDec.linFixedDirs[0];
				const Vector3D<> x = _spring->rowToDir[0];
				const Vector3D<> y = _spring->rowToDir[1];
				const Vector3D<> z = normalize(cross(x,y));
				pRcLin = Rotation3D<>(normalize(cross(fixed,z)),fixed,z);
				_spring->cDec.modes[0] = RWPEConstraint::Free;
				_spring->cDec.modes[1] = RWPEConstraint::Free;
				_spring->cDec.modes[2] = RWPEConstraint::Velocity;
				changed = true;
			} else if (_spring->linComp == 3 && _spring->cDec.linFixedDirs.size() == 2) {
				const Vector3D<> fixedA = _spring->cDec.linFixedDirs[0];
				const Vector3D<> fixedB = _spring->cDec.linFixedDirs[1];
				const Vector3D<> notFixed = normalize(cross(fixedA,fixedB));
				const Vector3D<> x = _spring->rowToDir[0];
				const Vector3D<> y = _spring->rowToDir[1];
				const Vector3D<> z = _spring->rowToDir[2];
				pRcLin = EAA<>(notFixed,Vector3D<>::x()).toRotation3D()*Rotation3D<>(x,y,z);
				_spring->cDec.modes[0] = RWPEConstraint::Free;
				_spring->cDec.modes[1] = RWPEConstraint::Velocity;
				_spring->cDec.modes[2] = RWPEConstraint::Velocity;
				changed = true;
			} else if (_spring->linComp == 3 && _spring->cDec.linFixedDirs.size() == 1) {
				const Vector3D<> fixed = _spring->cDec.linFixedDirs[0];
				const Vector3D<> x = _spring->rowToDir[0];
				const Vector3D<> y = _spring->rowToDir[1];
				const Vector3D<> z = _spring->rowToDir[2];
				pRcLin = EAA<>(fixed,Vector3D<>::z()).toRotation3D()*Rotation3D<>(x,y,z);
				_spring->cDec.modes[0] = RWPEConstraint::Free;
				_spring->cDec.modes[1] = RWPEConstraint::Free;
				_spring->cDec.modes[2] = RWPEConstraint::Velocity;
				changed = true;
			}
			if (changed) {
				cRcLin = Kinematics::frameTframe(parentFrame,childFrame,rwstate).R()*pRcLin;
				_rotLinParent = pRcLin;
				_rotLinChild = cRcLin;
			}
		}
		if (_spring->angComp > 0 && _spring->cDec.angFixedDirs.size() > 0) {
			bool changed = false;
			if (_spring->angComp == _spring->cDec.angFixedDirs.size()) {
				_spring->cDec.modes[3] = RWPEConstraint::Velocity;
				_spring->cDec.modes[4] = RWPEConstraint::Velocity;
				_spring->cDec.modes[5] = RWPEConstraint::Velocity;
			} else if (_spring->angComp == 2) {
				const Vector3D<> fixed = _spring->cDec.angFixedDirs[0];
				const Vector3D<> x = _spring->rowToDir[_spring->linComp+0];
				const Vector3D<> y = _spring->rowToDir[_spring->linComp+1];
				const Vector3D<> z = normalize(cross(x,y));
				pRcAng = Rotation3D<>(normalize(cross(fixed,z)),fixed,z);
				_spring->cDec.modes[3] = RWPEConstraint::Free;
				_spring->cDec.modes[4] = RWPEConstraint::Free;
				_spring->cDec.modes[5] = RWPEConstraint::Velocity;
				changed = true;
			} else if (_spring->angComp == 3 && _spring->cDec.angFixedDirs.size() == 2) {
				const Vector3D<> fixedA = _spring->cDec.angFixedDirs[0];
				const Vector3D<> fixedB = _spring->cDec.angFixedDirs[1];
				const Vector3D<> notFixed = normalize(cross(fixedA,fixedB));
				const Vector3D<> x = _spring->rowToDir[_spring->linComp+0];
				const Vector3D<> y = _spring->rowToDir[_spring->linComp+1];
				const Vector3D<> z = _spring->rowToDir[_spring->linComp+2];
				pRcAng = EAA<>(notFixed,Vector3D<>::x()).toRotation3D()*Rotation3D<>(x,y,z);
				_spring->cDec.modes[3] = RWPEConstraint::Free;
				_spring->cDec.modes[4] = RWPEConstraint::Velocity;
				_spring->cDec.modes[5] = RWPEConstraint::Velocity;
				changed = true;
			} else if (_spring->angComp == 3 && _spring->cDec.angFixedDirs.size() == 1) {
				const Vector3D<> fixed = _spring->cDec.angFixedDirs[0];
				const Vector3D<> x = _spring->rowToDir[_spring->linComp+0];
				const Vector3D<> y = _spring->rowToDir[_spring->linComp+1];
				const Vector3D<> z = _spring->rowToDir[_spring->linComp+2];
				pRcAng = EAA<>(fixed,Vector3D<>::z()).toRotation3D()*Rotation3D<>(x,y,z);
				_spring->cDec.modes[3] = RWPEConstraint::Free;
				_spring->cDec.modes[4] = RWPEConstraint::Free;
				_spring->cDec.modes[5] = RWPEConstraint::Velocity;
				changed = true;
			}
			if (changed) {
				cRcAng = Kinematics::frameTframe(parentFrame,childFrame,rwstate).R()*pRcAng;
				_rotAngParent = pRcAng;
				_rotAngChild = cRcAng;
			}
		}
	}

	const Transform3D<> wTparent = Kinematics::worldTframe(parentFrame,rwstate);
	const Transform3D<> parentTconstraint = _rwConstraint->getTransform();
	const Transform3D<> wTconstraint = wTparent*parentTconstraint;
	const Transform3D<> wTchild = Kinematics::worldTframe(childFrame,rwstate);

	// Find the current relative position and velocity in reduced space.
	const Eigen::VectorXd Xcur = getX(wTchild, wTconstraint, rwstate);
	const Eigen::VectorXd XcurD = getXd(wTparent, wTconstraint, islandState, rwstate);

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
	const Vector3D<> cartForceW = wTconstraint.R()*cart.first;
	const Vector3D<> cartTorqueW = wTconstraint.R()*cart.second;
	applyWrench(islandState,Wrench6D<>(cartForceW,cartTorqueW));
}

void RWPERWConstraint::reset(RWPEIslandState &islandState, const State &rwstate) {
	const Frame* const frameP = _rwConstraint->getBody1()->getBodyFrame();
	const Frame* const frameC = _rwConstraint->getBody2()->getBodyFrame();
	const Vector3D<> comP = _rwConstraint->getBody1()->getInfo().masscenter;
	const Vector3D<> comC = _rwConstraint->getBody2()->getInfo().masscenter;
	Transform3D<> pTc = _rwConstraint->getTransform();
	Transform3D<> cTc = Kinematics::frameTframe(frameC,frameP,rwstate)*pTc;
	pTc.P() -= comP;
	cTc.P() -= comC;
	_pTc = pTc;
	_cTc = cTc;
	_posParent = _pTc.P();
	_posChild = _cTc.P();
	_rotLinParent = _pTc.R();
	_rotLinChild = _cTc.R();
	_rotAngParent = _pTc.R();
	_rotAngChild = _cTc.R();
	if (_spring != NULL)
		delete _spring;
	_spring = NULL;
}

void RWPERWConstraint::step(RWPEIslandState &islandState, const State &rwstate, double h) {
}

std::vector<RWPEConstraint::Mode> RWPERWConstraint::getConstraintModes() const {
	if (_spring != NULL)
		return _spring->cDec.modes;
	else
		return _modes;
}

std::size_t RWPERWConstraint::getDimVelocity() const {
	if (_spring != NULL)
		return _dimVel+_spring->cDec.linFixedDirs.size()+_spring->cDec.angFixedDirs.size();
	else
		return _dimVel;
}

std::size_t RWPERWConstraint::getDimWrench() const {
	return 0;
}

std::size_t RWPERWConstraint::getDimFree() const {
	if (_spring != NULL)
		return _spring->cDec.freeDirs.size();
	else
		return _dimFree;
}

void RWPERWConstraint::createSpring() {
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
}

void RWPERWConstraint::deleteSpring() {
	delete _spring;
	_spring = NULL;
	_posParent = _pTc.P();
	_posChild = _cTc.P();
	_rotLinParent = _pTc.R();
	_rotLinChild = _cTc.R();
	_rotAngParent = _pTc.R();
	_rotAngChild = _cTc.R();
}

Eigen::VectorXd RWPERWConstraint::getX(const Transform3D<> &wTchild, const Transform3D<> &wTconstraint, const State &state) const {
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

Eigen::VectorXd RWPERWConstraint::getXd(const Transform3D<> &wTparent, const Transform3D<> &wTconstraint, const RWPEIslandState& islandState, const State &rwstate) const {
	const VelocityScrew6D<> velP = getParent()->getVelocityW(rwstate,islandState);
	const VelocityScrew6D<> velC = getChild()->getVelocityW(rwstate,islandState);
	const Vector3D<> linVelP = velP.linear();
	const Vector3D<> linVelC = velC.linear();
	const Vector3D<> angVelC = velC.angular().angle()*velC.angular().axis();
	const EAA<> angVelCon = getVelocityParentW(islandState,rwstate).angular();
	const Vector3D<> angVelConVec = angVelCon.angle()*angVelCon.axis();
	const VelocityScrew6D<> relVelP(linVelC-linVelP, EAA<>(angVelC-angVelConVec));
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

std::pair<Vector3D<>, Vector3D<> > RWPERWConstraint::toCartesian(const Eigen::VectorXd &reduced) const {
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

std::vector<RWPEConstraint::Mode> RWPERWConstraint::getModes(rw::common::Ptr<const Constraint> constraint) {
	std::vector<RWPEConstraint::Mode> modes;
	for(std::size_t i = 0; i < 6; i++)
		modes.push_back(Velocity);
	switch(constraint->getType()) {
	case Constraint::Fixed:
		break;
	case Constraint::Prismatic:
		modes[2] = Free;
		break;
	case Constraint::Revolute:
		modes[5] = Free;
		break;
	case Constraint::Universal:
		modes[3] = Free;
		modes[4] = Free;
		break;
	case Constraint::Spherical:
		modes[3] = Free;
		modes[4] = Free;
		modes[5] = Free;
		break;
	case Constraint::Piston:
		modes[2] = Free;
		modes[5] = Free;
		break;
	case Constraint::PrismaticRotoid:
		modes[2] = Free;
		modes[3] = Free;
		break;
	case Constraint::PrismaticUniversal:
		modes[2] = Free;
		modes[3] = Free;
		modes[4] = Free;
		break;
	case Constraint::Free:
		modes[0] = Free;
		modes[1] = Free;
		modes[2] = Free;
		modes[3] = Free;
		modes[4] = Free;
		modes[5] = Free;
		break;
	default:
		RW_THROW("RWPERWConstraint could not be created - type of Constraint is unknown!");
		break;
	}
	return modes;
}

void RWPERWConstraint::decomposeCompliance(const Eigen::MatrixXd &compliance, const Spring &spring, ComplianceDecomposition &dec) {
	// Do SVD on compliance matrix
	Eigen::MatrixXd U;
	Eigen::MatrixXd V;
	Eigen::VectorXd sigmaTmp;
	LinearAlgebra::svd(compliance,U,sigmaTmp,V);
	// Zero out close to singular directions
	const Eigen::VectorXd sigma = (sigmaTmp.array().abs() < RWPE_SPRING_EIGENVALUE_SQRT_THRESHOLD).select(0, sigmaTmp);
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
		if (std::fabs((double)sigma(i)) == 0)
			vec = V.col(i);
		else
			vec = U.col(i);
		bool lin = false;
		bool ang = false;
		std::vector<double> linVec(spring.linComp,0);
		std::vector<double> angVec(spring.angComp,0);
		std::vector<double> freeVec(spring.linComp+spring.angComp,0);

		for (Eigen::DenseIndex j = 0; j < vec.rows(); j++) {
			if (std::fabs((double)vec(j)) > 0) {
				if (j < spring.linComp) {
					lin = true;
					linVec[j] = vec(j);
				} else {
					ang = true;
					angVec[j-spring.linComp] = vec(j);
				}
				freeVec[j] = vec(j);
			}
		}

		if (std::fabs((double)sigma(i)) == 0) {
			if (lin && ang)
				RW_THROW("Null space of compliance matrix spans vectors that are mixed linear and angular.");
			if (lin)
				linFixed.push_back(linVec);
			if (ang)
				angFixed.push_back(angVec);
		} else {
			free.push_back(freeVec);
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
