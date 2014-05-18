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

#include "TNTBodyConstraintManager.hpp"
#include "TNTBody.hpp"
#include "TNTFixedBody.hpp"
#include "TNTKinematicBody.hpp"
#include "TNTRigidBody.hpp"

#include "TNTConstraint.hpp"
#include "TNTRWConstraint.hpp"
#include "TNTIslandState.hpp"

#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

TNTBodyConstraintManager::TNTBodyConstraintManager()
{
}

TNTBodyConstraintManager::~TNTBodyConstraintManager() {
}

void TNTBodyConstraintManager::initFromDWC(const rw::common::Ptr<const DynamicWorkCell>& dwc) {
	std::map<rwsim::dynamics::Body*, const TNTBody*> rwBodyToBody;
	const std::vector<Body::Ptr> bodies = dwc->getBodies();
	BOOST_FOREACH(Body::Ptr body, bodies) {
		const TNTBody* tntbody;
		if(RigidBody::Ptr rbody = body.cast<RigidBody>() )
			tntbody = new TNTRigidBody(rbody);
		else if(FixedBody::Ptr fbody = body.cast<FixedBody>())
			tntbody = new TNTFixedBody(fbody);
		else if(KinematicBody::Ptr kbody = body.cast<KinematicBody>())
			tntbody = new TNTKinematicBody(kbody);
		else
			RW_THROW("BodyContactManager (initFromDWC): The type for body \"" << body->getName() << "\" is unsupported!");
		addBody(tntbody);
		rwBodyToBody[body.get()] = tntbody;
	}
	const std::vector<Constraint::Ptr> constraints = dwc->getConstraints();
	BOOST_FOREACH(Constraint::Ptr constraint, constraints) {
		const TNTBody* const parent = rwBodyToBody[constraint->getBody1()];
		const TNTBody* const child = rwBodyToBody[constraint->getBody2()];
		if (parent == NULL)
			RW_THROW("BodyContactManager (initFromDWC): The body body \"" << parent->get()->getName() << "\" is unknown for constraint " << constraint->getName() << "!");
		if (child == NULL)
			RW_THROW("BodyContactManager (initFromDWC): The body body \"" << child->get()->getName() << "\" is unknown for constraint " << constraint->getName() << "!");
		TNTRWConstraint* const tntconstraint = new TNTRWConstraint(constraint,parent,child);
		addConstraint(tntconstraint);
	}
}

TNTBodyConstraintManager::BodyList TNTBodyConstraintManager::getBodies() const {
	return _allBodies;
}

TNTBodyConstraintManager::DynamicBodyList TNTBodyConstraintManager::getDynamicBodies() const {
	return _dynamicBodies;
}

TNTBodyConstraintManager::KinematicBodyList TNTBodyConstraintManager::getKinematicBodies() const {
	return _kinematicBodies;
}

TNTBodyConstraintManager::ConstraintList TNTBodyConstraintManager::getPersistentConstraints() const {
	return _constraints;
}

TNTBodyConstraintManager::ConstraintList TNTBodyConstraintManager::getConstraints(const TNTIslandState& state) const {
	TNTBodyConstraintManager::ConstraintList list = _constraints;
	const TNTBodyConstraintManager::ConstraintList templist = state.getTemporaryConstraints();
	list.insert(list.end(),templist.begin(),templist.end());
	return list;
}

TNTBodyConstraintManager::ConstraintListConst TNTBodyConstraintManager::getConstraints(const TNTBody* body, const TNTIslandState& state) const {
	TNTBodyConstraintManager::ConstraintListConst list;
	{
		std::map<const TNTBody*, ConstraintListConst>::const_iterator it = _bodyToConstraints.find(body);
		if (it != _bodyToConstraints.end())
			list = (*it).second;
	}
	const TNTBodyConstraintManager::ConstraintListConst templist = state.getTemporaryConstraints(body);
	list.insert(list.end(),templist.begin(),templist.end());
	return list;
}

TNTBodyConstraintManager::ConstraintListConst TNTBodyConstraintManager::getConstraints(const TNTBody* bodyA, const TNTBody* bodyB, const TNTIslandState& state) const {
	TNTBodyConstraintManager::ConstraintListConst list;
	{
		std::map<const TNTBody*, ConstraintListConst>::const_iterator it = _bodyToConstraints.find(bodyA);
		if (it != _bodyToConstraints.end()) {
			const ConstraintListConst constraintsA = it->second;
			BOOST_FOREACH(const TNTConstraint* constraint, constraintsA) {
				if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
						(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
					list.push_back(constraint);
				}
			}
		}
	}
	const TNTBodyConstraintManager::ConstraintListConst templist = state.getTemporaryConstraints(bodyA);
	BOOST_FOREACH(const TNTConstraint* constraint, templist) {
		if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
				(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
			list.push_back(constraint);
		}
	}
	return list;
}

TNTBodyConstraintManager::ConstraintList TNTBodyConstraintManager::getTemporaryConstraints(const TNTIslandState* state) const {
	return state->getTemporaryConstraints();
}

void TNTBodyConstraintManager::addBody(const TNTBody* const body) {
	_allBodies.push_back(body);

	if (const TNTRigidBody* rbody = dynamic_cast<const TNTRigidBody*>(body))
		_dynamicBodies.push_back(rbody);
	else if (const TNTKinematicBody* kbody = dynamic_cast<const TNTKinematicBody*>(body))
		_kinematicBodies.push_back(kbody);

	_frameToBody[body->get()->getBodyFrame()] = body;
}

void TNTBodyConstraintManager::addConstraint(TNTConstraint* constraint) {
	_constraints.push_back(constraint);
	RW_ASSERT(constraint->getParent() != NULL);
	RW_ASSERT(constraint->getChild() != NULL);
	_bodyToConstraints[constraint->getParent()].push_back(constraint);
	_bodyToConstraints[constraint->getChild()].push_back(constraint);
}

void TNTBodyConstraintManager::addTemporaryConstraint(TNTConstraint* constraint, TNTIslandState& state) const {
	state.addTemporaryConstraint(constraint);
}

void TNTBodyConstraintManager::removeBody(const TNTBody* const body) {
	_allBodies.remove(body);

	if (const TNTRigidBody* rbody = dynamic_cast<const TNTRigidBody*>(body))
		_dynamicBodies.remove(rbody);
	else if (const TNTKinematicBody* kbody = dynamic_cast<const TNTKinematicBody*>(body))
		_kinematicBodies.remove(kbody);
}

void TNTBodyConstraintManager::removeConstraint(TNTConstraint* constraint) {
	_constraints.remove(constraint);
	_bodyToConstraints[constraint->getParent()].remove(constraint);
	_bodyToConstraints[constraint->getChild()].remove(constraint);
}

void TNTBodyConstraintManager::removeTemporaryConstraint(const TNTConstraint* constraint, TNTIslandState& state) const {
	state.removeTemporaryConstraint(constraint);
}

void TNTBodyConstraintManager::clearTemporaryConstraints(TNTIslandState& state) const {
	state.clearTemporaryConstraints();
}

const TNTBody* TNTBodyConstraintManager::getBody(const rw::kinematics::Frame* frame) const {
	std::map<const rw::kinematics::Frame*, const TNTBody*>::const_iterator it = _frameToBody.find(frame);
	if (it != _frameToBody.end())
		return (*it).second;
	return NULL;
}
