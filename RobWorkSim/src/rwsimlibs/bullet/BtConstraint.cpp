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

#include "BtConstraint.hpp"
#include "BtBody.hpp"
#include "BtUtil.hpp"

#include <rwsim/dynamics/Constraint.hpp>

#include <bullet/BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#include <bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.h>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsim::dynamics;
using namespace rwsimlibs::bullet;

BtConstraint::BtConstraint(rw::common::Ptr<const Constraint> constraint, const BtBody* const parent, const BtBody* const child, btDynamicsWorld* btWorld):
	_rwConstraint(constraint),
	_parent(parent),
	_child(child),
	_btDynamicsWorld(btWorld),
	_btConstraint(NULL),
	_feedback(NULL)
{
	createJoint();
}

BtConstraint::~BtConstraint() {
	if (_feedback)
		delete _feedback;
	destroyJoint();
}

btTypedConstraint* BtConstraint::getBtConstraint() const {
	return _btConstraint;
}

rw::common::Ptr<const Constraint> BtConstraint::getRWConstraint() const {
	return _rwConstraint;
}

btRigidBody* BtConstraint::getBtParent() const {
	return _parent->getBulletBody();
}

btRigidBody* BtConstraint::getBtChild() const {
	return _child->getBulletBody();
}

Vector3D<> BtConstraint::getRefA() const {
	//const Constraint::ConstraintType type = _rwConstraint->getType();
	//if (type == Constraint::Fixed) {
	return _parent->getWorldTcom().P();
	//}
}

Vector3D<> BtConstraint::getRefB() const {
	return _parent->getWorldTcom().P();
}

void BtConstraint::createJoint() {
	// Note: the split impulse option is not used for constraints (only possible for contacts)
	// hence constraints will be soft (Baumgarte stabilization eith ERP and CFM parameters)!
	btRigidBody& first = *_parent->getBulletBody();
	btRigidBody& second = *_child->getBulletBody();
	const Transform3D<> parentTconstraint = _rwConstraint->getTransform();
	const Transform3D<>& pTcom = _parent->getBodyTcom();
	const Transform3D<> comTconstraint = inverse(pTcom)*parentTconstraint;
	const Transform3D<> frameInA = comTconstraint;
	const Transform3D<>& wTc_com = _child->getWorldTcom();
	const Transform3D<>& wTp_com = _parent->getWorldTcom();
	const Transform3D<> frameInB = inverse(wTc_com)*wTp_com*comTconstraint;
	const Constraint::SpringParams spring = _rwConstraint->getSpringParams();
	static const bool useLinearReferenceFrameA = true;
	const Constraint::ConstraintType type = _rwConstraint->getType();

	if (spring.enabled) {
		if (!spring.compliance.isDiagonal(1e-8))
			RW_THROW("Bullet does not support springs with non-diagonal compliance!");
		if (!spring.damping.isDiagonal(1e-8))
			RW_THROW("Bullet does not support springs with non-diagonal damping!");
		if (spring.damping.diagonal().maxCoeff() > 0)
			RW_THROW("Bullet spring damping is currently not implemented.");
	}

	/*if (type == Constraint::Fixed) {
		btFixedConstraint* const btConstraint = new btFixedConstraint(second, first, BtUtil::makeBtTransform(frameInB), BtUtil::makeBtTransform(frameInA));
		_btConstraint = btConstraint;
	} else*/
	if (type == Constraint::Fixed) {
		// Note: from Bullet 2.82 there is a dedicated fixed constraint type!
		btGeneric6DofConstraint* const btConstraint = new btGeneric6DofConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
		// Fix all linear and angular axes by setting higher and lower limits equal
		for (std::size_t i = 0; i < 6; i++) {
			btConstraint->setParam(BT_CONSTRAINT_CFM,0,i);
			btConstraint->setParam(BT_CONSTRAINT_STOP_CFM,0,i); // Default is m_globalCfm set in BtSimulator (which is default 0)
			btConstraint->setParam(BT_CONSTRAINT_STOP_ERP,0.2,i); // Default is m_erp set in BtSimulator (which is default 0.2)
			if (i < 3) {
				btConstraint->getTranslationalLimitMotor()->m_enableMotor[i] = true;
				btConstraint->getTranslationalLimitMotor()->m_maxMotorForce[i] = SIMD_INFINITY;
				//btConstraint->getTranslationalLimitMotor()->m_currentLimit[i] = 0;
				//btConstraint->getTranslationalLimitMotor()->m_lowerLimit[i] = -SIMD_INFINITY;
				//btConstraint->getTranslationalLimitMotor()->m_upperLimit[i] = SIMD_INFINITY;
			} else {
				btConstraint->setLimit(i,0,0);
				btConstraint->getRotationalLimitMotor(i)->m_enableMotor = true;
				btConstraint->getRotationalLimitMotor(i)->m_maxMotorForce = SIMD_INFINITY;
				//btConstraint->getRotationalLimitMotor(i)->m_limitSoftness = 0;
				//btConstraint->getRotationalLimitMotor(i)->m_currentLimit = 0;
				//btConstraint->getRotationalLimitMotor(i)->m_loLimit = -SIMD_INFINITY;
				//btConstraint->getRotationalLimitMotor(i)->m_hiLimit = SIMD_INFINITY;
			}
			//btConstraint->getTranslationalLimitMotor()->m_limitSoftness = 0;
		}
		_btConstraint = btConstraint;
	} else if (type == Constraint::Prismatic) {
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btConstraint = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			btConstraint->enableSpring(2,true);
			btConstraint->setStiffness(2,1./spring.compliance(0,0));
			btConstraint->setDamping(2,spring.damping(0,0));
			// Now limits are used to make translation along z possible
			btConstraint->setLinearLowerLimit(btVector3(0,0,1));
			btConstraint->setLinearUpperLimit(btVector3(0,0,-1));
			btConstraint->setAngularLowerLimit(btVector3(0,0,0));
			btConstraint->setAngularUpperLimit(btVector3(0,0,0));
			_btConstraint = btConstraint;
		} else {
			// Rotation is about x in Bullet and around z in RobWork, so we must apply rotation.
			const Rotation3D<> xRz(0, 0, -1, 0, 1, 0, 1, 0, 0); // -90 about y
			const Transform3D<> btTransformA(frameInA.P(),frameInA.R()*xRz);
			const Transform3D<> btTransformB(frameInB.P(),frameInB.R()*xRz);
			btSliderConstraint* const btConstraint = new btSliderConstraint(first, second, BtUtil::makeBtTransform(btTransformA), BtUtil::makeBtTransform(btTransformB), useLinearReferenceFrameA);
			// btSliderConstraint is a piston, angular part is fixed, and linear part is free
			btConstraint->setLowerLinLimit(1);
			btConstraint->setUpperLinLimit(-1);
			btConstraint->setLowerAngLimit(0);
			btConstraint->setUpperAngLimit(0);
			_btConstraint = btConstraint;
		}
	} else if (type == Constraint::Revolute) {
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btConstraint = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			btConstraint->enableSpring(5,true);
			btConstraint->setStiffness(5,1./spring.compliance(0,0));
			btConstraint->setDamping(5,spring.damping(0,0));
			// Now limits are used to make rotation around z possible
			btConstraint->setLinearLowerLimit(btVector3(0,0,0));
			btConstraint->setLinearUpperLimit(btVector3(0,0,0));
			btConstraint->setAngularLowerLimit(btVector3(0,0,1));
			btConstraint->setAngularUpperLimit(btVector3(0,0,-1));
			_btConstraint = btConstraint;
		} else {
			_btConstraint = new btHingeConstraint(first, second,
					BtUtil::makeBtVector(frameInA.P()), BtUtil::makeBtVector(frameInB.P()),
					BtUtil::makeBtVector(frameInA.R().getCol(2)), BtUtil::makeBtVector(frameInB.R().getCol(2)),
					useLinearReferenceFrameA);
		}
	} else if (type == Constraint::Universal) {
		btGeneric6DofConstraint* btConstraint;
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btSpring = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			btConstraint = btSpring;
			unsigned int dirs[] = {3,4};
			for (unsigned int dof = 0; dof < 2; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btSpring->enableSpring(dirs[dof],true);
					btSpring->setStiffness(dirs[dof],1./spring.compliance(dof,dof));
					btSpring->setDamping(dirs[dof],spring.damping(dof,dof));
				}
			}
		} else {
			btConstraint = new btGeneric6DofConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
		}
		// Now limits are used to make rotation about x and y free
		btConstraint->setLinearLowerLimit(btVector3(0,0,0));
		btConstraint->setLinearUpperLimit(btVector3(0,0,0));
		btConstraint->setAngularLowerLimit(btVector3(1,1,0));
		btConstraint->setAngularUpperLimit(btVector3(-1,-1,0));
		_btConstraint = btConstraint;
	} else if (type == Constraint::Spherical) {
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btConstraint = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			unsigned int dirs[] = {3,4,5};
			for (unsigned int dof = 0; dof < 3; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btConstraint->enableSpring(dirs[dof],true);
					btConstraint->setStiffness(dirs[dof],1./spring.compliance(dof,dof));
					btConstraint->setDamping(dirs[dof],spring.damping(dof,dof));
				}
			}
			// Now limits are used to make rotation possible
			btConstraint->setLinearLowerLimit(btVector3(0,0,0));
			btConstraint->setLinearUpperLimit(btVector3(0,0,0));
			btConstraint->setAngularLowerLimit(btVector3(1,1,1));
			btConstraint->setAngularUpperLimit(btVector3(-1,-1,-1));
			_btConstraint = btConstraint;
		} else {
			_btConstraint = new btPoint2PointConstraint(first, second, BtUtil::makeBtVector(frameInA.P()), BtUtil::makeBtVector(frameInB.P()));
		}
	} else if (type == Constraint::Piston) {
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btConstraint = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			unsigned int dirs[] = {2,5};
			for (unsigned int dof = 0; dof < 2; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btConstraint->enableSpring(dirs[dof],true);
					btConstraint->setStiffness(dirs[dof],1./spring.compliance(dof,dof));
					btConstraint->setDamping(dirs[dof],spring.damping(dof,dof));
				}
			}
			// Now limits are used to make rotation possible around z, and to make linear motion possible along z
			btConstraint->setLinearLowerLimit(btVector3(0,0,1));
			btConstraint->setLinearUpperLimit(btVector3(0,0,-1));
			btConstraint->setAngularLowerLimit(btVector3(0,0,1));
			btConstraint->setAngularUpperLimit(btVector3(0,0,-1));
			_btConstraint = btConstraint;
		} else {
			// Rotation is about x in Bullet and around z in RobWork, so we must apply rotation.
			const Rotation3D<> xRz(0, 0, -1, 0, 1, 0, 1, 0, 0); // -90 about y
			const Transform3D<> btTransformA(frameInA.P(),frameInA.R()*xRz);
			const Transform3D<> btTransformB(frameInB.P(),frameInB.R()*xRz);
			btSliderConstraint* const btConstraint = new btSliderConstraint(first, second, BtUtil::makeBtTransform(btTransformA), BtUtil::makeBtTransform(btTransformB), useLinearReferenceFrameA);
			// btSliderConstraint is a piston, we set angular and linear part as free
			btConstraint->setLowerLinLimit(1);
			btConstraint->setUpperLinLimit(-1);
			btConstraint->setLowerAngLimit(1);
			btConstraint->setUpperAngLimit(-1);
			_btConstraint = btConstraint;
		}
	} else if (type == Constraint::PrismaticRotoid) {
		btGeneric6DofConstraint* btConstraint;
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btSpring = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			btConstraint = btSpring;
			unsigned int dirs[] = {2,3};
			for (unsigned int dof = 0; dof < 2; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btSpring->enableSpring(dirs[dof],true);
					btSpring->setStiffness(dirs[dof],1./spring.compliance(dof,dof));
					btSpring->setDamping(dirs[dof],spring.damping(dof,dof));
				}
			}
		} else {
			btConstraint = new btGeneric6DofConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
		}
		// Now limits are used to make translation along z possible, as well as rotation about x
		btConstraint->setLinearLowerLimit(btVector3(0,0,1));
		btConstraint->setLinearUpperLimit(btVector3(0,0,-1));
		btConstraint->setAngularLowerLimit(btVector3(1,0,0));
		btConstraint->setAngularUpperLimit(btVector3(-1,0,0));
		_btConstraint = btConstraint;
	} else if (type == Constraint::PrismaticUniversal) {
		btGeneric6DofConstraint* btConstraint;
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btSpring = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			btConstraint = btSpring;
			unsigned int dirs[] = {2,3,4};
			for (unsigned int dof = 0; dof < 3; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btSpring->enableSpring(dirs[dof],true);
					btSpring->setStiffness(dirs[dof],1./spring.compliance(dof,dof));
					btSpring->setDamping(dirs[dof],spring.damping(dof,dof));
				}
			}
		} else {
			btConstraint = new btGeneric6DofConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
		}
		// Now limits are used to make translation along z possible, as well as rotation about x and y
		btConstraint->setLinearLowerLimit(btVector3(0,0,1));
		btConstraint->setLinearUpperLimit(btVector3(0,0,-1));
		btConstraint->setAngularLowerLimit(btVector3(1,1,0));
		btConstraint->setAngularUpperLimit(btVector3(-1,-1,0));
		_btConstraint = btConstraint;
	} else if (type == Constraint::Free) {
		if (spring.enabled) {
			btGeneric6DofSpringConstraint* const btConstraint = new btGeneric6DofSpringConstraint(first, second, BtUtil::makeBtTransform(frameInA), BtUtil::makeBtTransform(frameInB), useLinearReferenceFrameA);
			for (unsigned int dof = 0; dof < 6; dof++) {
				if (spring.compliance(dof,dof) > 0) {
					btConstraint->enableSpring(dof,true);
					btConstraint->setStiffness(dof,1./spring.compliance(dof,dof));
					if (spring.damping(dof,dof) <= 0.)
						btConstraint->setDamping(dof,1.); // between 0 and 1 (1 is undamped)
					else
						btConstraint->setDamping(dof,0); // how to set this?!
					btConstraint->setLinearLowerLimit(btVector3(1,1,1));
					btConstraint->setLinearUpperLimit(btVector3(-1,-1,-1));
				}
			}
			for (unsigned int dof = 0; dof < 3; dof++) {
				const Transform3D<> constraintTchildP = inverse(comTconstraint)*inverse(wTp_com)*wTc_com;
				btConstraint->setEquilibriumPoint(dof,-constraintTchildP.P()[dof]);
			}
			_btConstraint = btConstraint;
		}
	} else {
		RW_THROW("Unsupported Constraint type!");
	}
	if (_btConstraint != NULL)
		_btDynamicsWorld->addConstraint(_btConstraint, true);
}

void BtConstraint::destroyJoint() {
	if (_btConstraint != NULL) {
		_btDynamicsWorld->removeConstraint(_btConstraint); // only do this if addConstraint is called with disableCollisionsBetweenLinkedBodies=true (bug?)
		_btConstraint = NULL;
	}
}

void BtConstraint::update(const double dt, State& state) {
	//_child->getBulletBody()->applyCentralForce(btVector3(0,0,0.06*9.82));
	//std::cout << "applied impulse b: " << _btConstraint->getAppliedImpulse() << " " << _btConstraint->getAppliedImpulse()/dt << std::endl;
}

void BtConstraint::postUpdate(rw::kinematics::State& state) {
	//std::cout << "applied impulse a: " << _btConstraint->getAppliedImpulse() << " " << _btConstraint->getAppliedImpulse()/0.0052 << std::endl;
}

btJointFeedback* BtConstraint::getFeedback() {
	if (!_feedback)
		_feedback = new btJointFeedback();
	return _feedback;
}
