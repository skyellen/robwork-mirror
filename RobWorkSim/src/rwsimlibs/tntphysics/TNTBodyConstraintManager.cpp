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

TNTBodyConstraintManager::TNTBodyConstraintManager():
	_parent(NULL)
{
}

TNTBodyConstraintManager::TNTBodyConstraintManager(const TNTBodyConstraintManager* parent):
	_parent(parent)
{
}

TNTBodyConstraintManager::~TNTBodyConstraintManager() {
	if (_parent == NULL) {
		BOOST_FOREACH(const TNTBody* const body, _allBodies) {
			delete body;
		}
		BOOST_FOREACH(const TNTConstraint* const constraint, _constraints) {
			delete constraint;
		}
	}
	_allBodies.clear();
	_dynamicBodies.clear();
	_kinematicBodies.clear();
	_frameToBody.clear();
	_constraints.clear();
	_bodyToConstraints.clear();
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
	if (_parent == NULL) {
		const TNTBodyConstraintManager::ConstraintList templist = state.getTemporaryConstraints();
		list.insert(list.end(),templist.begin(),templist.end());
	}
	return list;
}

TNTBodyConstraintManager::ConstraintListConst TNTBodyConstraintManager::getConstraints(const TNTBody* body, const TNTIslandState& state) const {
	TNTBodyConstraintManager::ConstraintListConst list;
	{
		std::map<const TNTBody*, ConstraintListConst>::const_iterator it = _bodyToConstraints.find(body);
		if (it != _bodyToConstraints.end())
			list = (*it).second;
	}
	if (_parent == NULL) {
		const TNTBodyConstraintManager::ConstraintListConst templist = state.getTemporaryConstraints(body);
		list.insert(list.end(),templist.begin(),templist.end());
	}
	return list;
}

TNTBodyConstraintManager::ConstraintList TNTBodyConstraintManager::getConstraints(const TNTBody* bodyA, const TNTBody* bodyB) const {
	TNTBodyConstraintManager::ConstraintList list;
	BOOST_FOREACH(TNTConstraint* constraint, _constraints) {
		if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
				(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
			list.push_back(constraint);
		}
	}
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
	if (_parent == NULL) {
		const TNTBodyConstraintManager::ConstraintListConst templist = state.getTemporaryConstraints(bodyA);
		BOOST_FOREACH(const TNTConstraint* constraint, templist) {
			if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
					(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
				list.push_back(constraint);
			}
		}
	}
	return list;
}

TNTBodyConstraintManager::ConstraintList TNTBodyConstraintManager::getTemporaryConstraints(const TNTIslandState* state) const {
	if (_parent == NULL)
		return state->getTemporaryConstraints();
	else
		return ConstraintList();
}

void TNTBodyConstraintManager::addBody(const TNTBody* const body) {
	_allBodies.push_back(body);

	if (const TNTRigidBody* rbody = dynamic_cast<const TNTRigidBody*>(body))
		_dynamicBodies.push_back(rbody);
	else if (const TNTKinematicBody* kbody = dynamic_cast<const TNTKinematicBody*>(body))
		_kinematicBodies.push_back(kbody);

	_frameToBody[body->get()->getBodyFrame()] = body;
}

void TNTBodyConstraintManager::addBodies(const BodyList& bodies) {
	BOOST_FOREACH(const TNTBody* const body, bodies) {
		addBody(body);
	}
}

void TNTBodyConstraintManager::addConstraint(TNTConstraint* constraint) {
	_constraints.push_back(constraint);
	RW_ASSERT(constraint->getParent() != NULL);
	RW_ASSERT(constraint->getChild() != NULL);
	_bodyToConstraints[constraint->getParent()].push_back(constraint);
	_bodyToConstraints[constraint->getChild()].push_back(constraint);
}

void TNTBodyConstraintManager::addConstraints(const ConstraintList& constraints) {
	_constraints.insert(_constraints.end(),constraints.begin(),constraints.end());
	BOOST_FOREACH(TNTConstraint* const constraint, constraints) {
		RW_ASSERT(constraint->getParent() != NULL);
		RW_ASSERT(constraint->getChild() != NULL);
		_bodyToConstraints[constraint->getParent()].push_back(constraint);
		_bodyToConstraints[constraint->getChild()].push_back(constraint);
	}
}

void TNTBodyConstraintManager::addTemporaryConstraint(TNTConstraint* constraint, TNTIslandState& state) const {
	if (_parent == NULL) {
		state.addTemporaryConstraint(constraint);
	} else {
		RW_THROW("TNTBodyConstraintManager (addTemporaryConstraint): as this manager represents a subcomponent, it is not allowed to add temporary constraints!");
	}
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
	if (_parent == NULL) {
		state.removeTemporaryConstraint(constraint);
	} else {
		RW_THROW("TNTBodyConstraintManager (removeTemporaryConstraint): as this manager represents a subcomponent, it is not allowed to remove temporary constraints!");
	}
}

void TNTBodyConstraintManager::clearTemporaryConstraints(TNTIslandState& state) const {
	if (_parent == NULL) {
		state.clearTemporaryConstraints();
	} else {
		RW_THROW("TNTBodyConstraintManager (clearTemporaryConstraints): as this manager represents a subcomponent, it is not allowed to remove temporary constraints!");
	}
}

const TNTBody* TNTBodyConstraintManager::getBody(const rw::kinematics::Frame* frame) const {
	std::map<const rw::kinematics::Frame*, const TNTBody*>::const_iterator it = _frameToBody.find(frame);
	if (it != _frameToBody.end())
		return (*it).second;
	return NULL;
}

bool TNTBodyConstraintManager::has(const TNTBody* body) const {
	BOOST_FOREACH(const TNTBody* const b, _allBodies) {
		if (b == body)
			return true;
	}
	return false;
}

std::set<TNTBodyConstraintManager*> TNTBodyConstraintManager::getDynamicComponents(const TNTIslandState& state) const {
	std::set<TNTBodyConstraintManager*> components;

	// Traverse all contacts and constraints and build initial components
	BOOST_FOREACH(TNTConstraint* const constraint, getConstraints(state)) {
		if (constraint->getDimVelocity() == 0)
			continue;
		const TNTRigidBody* const parent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const TNTRigidBody* const child = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<TNTBodyConstraintManager*>::iterator parentPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator childPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const TNTRigidBody* const b, (*it)->getDynamicBodies()) {
				if (parent == b)
					parentPos = it;
				if (child == b)
					childPos = it;
				if (parentPos != components.end() && childPos != components.end())
					break;
			}
		}
		if (parentPos == components.end() && childPos == components.end()) {
			// If not found in any components, a new one is added
			// (we already know that at least one of the bodies must be dynamic)
			TNTBodyConstraintManager* const newComponent = new TNTBodyConstraintManager(this);
			components.insert(newComponent);
			if (parent)
				newComponent->addBody(parent);
			if (child)
				newComponent->addBody(child);
			newComponent->addConstraint(constraint);
		} else if (parentPos == components.end() && childPos != components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (parent)
				(*childPos)->addBody(parent);
			(*childPos)->addConstraint(constraint);
		} else if (parentPos != components.end() && childPos == components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (child)
				(*parentPos)->addBody(child);
			(*parentPos)->addConstraint(constraint);
		} else if (parentPos != components.end() && childPos != components.end()) {
			// A component was found for both bodies
			(*parentPos)->addConstraint(constraint);
			if (parentPos != childPos) {
				// The two components are different, and are therefore merged into one
				(*parentPos)->addBodies((*childPos)->getBodies());
				(*parentPos)->addConstraints((*childPos)->getPersistentConstraints());
				delete (*childPos);
				components.erase(childPos);
			}
		}
	}

	// Go through each component and insert the non-dynamic bodies
	BOOST_FOREACH(TNTBodyConstraintManager* const component, components) {
		BOOST_FOREACH(TNTConstraint* const constraint, component->getPersistentConstraints()) {
			const TNTBody* const parent = dynamic_cast<const TNTBody*>(constraint->getParent());
			const TNTBody* const child = dynamic_cast<const TNTBody*>(constraint->getChild());
			const TNTBody* staticBody = NULL;
			if (!dynamic_cast<const TNTRigidBody*>(constraint->getParent())) {
				staticBody = parent;
			}
			if (!dynamic_cast<const TNTRigidBody*>(constraint->getChild())) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const TNTBody* const existingBody, component->getBodies()) {
				if (existingBody == staticBody) {
					found = true;
					break;
				}
			}
			if (!found) {
				component->addBody(staticBody);
			}
		}
	}

	return components;
}

std::set<TNTBodyConstraintManager*> TNTBodyConstraintManager::getDynamicComponents(const std::list<std::pair<const TNTBody*, const TNTBody*> >& pairs) const {
	std::set<TNTBodyConstraintManager*> components;

	// Traverse all constraints and build initial components
	BOOST_FOREACH(TNTConstraint* const constraint, getPersistentConstraints()) {
		//if (constraint->getDimVelocity() == 0)
		//	continue;
		const TNTRigidBody* const parent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const TNTRigidBody* const child = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<TNTBodyConstraintManager*>::iterator parentPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator childPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const TNTRigidBody* const b, (*it)->getDynamicBodies()) {
				if (parent == b)
					parentPos = it;
				if (child == b)
					childPos = it;
				if (parentPos != components.end() && childPos != components.end())
					break;
			}
		}
		if (parentPos == components.end() && childPos == components.end()) {
			// If not found in any components, a new one is added
			// (we already know that at least one of the bodies must be dynamic)
			TNTBodyConstraintManager* const newComponent = new TNTBodyConstraintManager(this);
			components.insert(newComponent);
			if (parent)
				newComponent->addBody(parent);
			if (child)
				newComponent->addBody(child);
			newComponent->addConstraint(constraint);
		} else if (parentPos == components.end() && childPos != components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (parent)
				(*childPos)->addBody(parent);
			(*childPos)->addConstraint(constraint);
		} else if (parentPos != components.end() && childPos == components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (child)
				(*parentPos)->addBody(child);
			(*parentPos)->addConstraint(constraint);
		} else if (parentPos != components.end() && childPos != components.end()) {
			// A component was found for both bodies
			(*parentPos)->addConstraint(constraint);
			if (parentPos != childPos) {
				// The two components are different, and are therefore merged into one
				(*parentPos)->addBodies((*childPos)->getBodies());
				(*parentPos)->addConstraints((*childPos)->getPersistentConstraints());
				delete (*childPos);
				components.erase(childPos);
			}
		}
	}

	// Now traverse the input list
	typedef std::pair<const TNTBody*, const TNTBody*> BodyPair;
	BOOST_FOREACH(const BodyPair& pair, pairs) {
		const TNTRigidBody* const parent = dynamic_cast<const TNTRigidBody*>(pair.first);
		const TNTRigidBody* const child = dynamic_cast<const TNTRigidBody*>(pair.second);
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<TNTBodyConstraintManager*>::iterator parentPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator childPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const TNTRigidBody* const b, (*it)->getDynamicBodies()) {
				if (parent == b)
					parentPos = it;
				if (child == b)
					childPos = it;
				if (parentPos != components.end() && childPos != components.end())
					break;
			}
		}
		if (parentPos == components.end() && childPos == components.end()) {
			// If not found in any components, a new one is added
			// (we already know that at least one of the bodies must be dynamic)
			TNTBodyConstraintManager* const newComponent = new TNTBodyConstraintManager(this);
			components.insert(newComponent);
			if (parent)
				newComponent->addBody(parent);
			if (child)
				newComponent->addBody(child);
		} else if (parentPos == components.end() && childPos != components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (parent)
				(*childPos)->addBody(parent);
		} else if (parentPos != components.end() && childPos == components.end()) {
			// A component was found for one of the bodies, and the other body is added to
			// this component (if it is not static)
			if (child)
				(*parentPos)->addBody(child);
		} else if (parentPos != components.end() && childPos != components.end()) {
			// A component was found for both bodies
			if (parentPos != childPos) {
				// The two components are different, and are therefore merged into one
				(*parentPos)->addBodies((*childPos)->getBodies());
				(*parentPos)->addConstraints((*childPos)->getPersistentConstraints());
				delete (*childPos);
				components.erase(childPos);
			}
		}
	}

	// Go through each component and insert the non-dynamic bodies
	BOOST_FOREACH(TNTBodyConstraintManager* const component, components) {
		BOOST_FOREACH(TNTConstraint* const constraint, component->getPersistentConstraints()) {
			const TNTBody* const parent = constraint->getParent();
			const TNTBody* const child = constraint->getChild();
			const TNTBody* staticBody = NULL;
			if (!dynamic_cast<const TNTRigidBody*>(parent)) {
				staticBody = parent;
			}
			if (!dynamic_cast<const TNTRigidBody*>(child)) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const TNTBody* const existingBody, component->getBodies()) {
				if (existingBody == staticBody) {
					found = true;
					break;
				}
			}
			if (!found) {
				component->addBody(staticBody);
			}
		}
		BOOST_FOREACH(const BodyPair& pair, pairs) {
			const TNTBody* const parent = pair.first;
			const TNTBody* const child = pair.second;
			const TNTBody* staticBody = NULL;
			if (!dynamic_cast<const TNTRigidBody*>(parent)) {
				staticBody = parent;
			}
			if (!dynamic_cast<const TNTRigidBody*>(child)) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const TNTBody* const existingBody, component->getBodies()) {
				if (existingBody == staticBody) {
					found = true;
					break;
				}
			}
			if (!found) {
				component->addBody(staticBody);
			}
		}
	}

	return components;
}

std::set<TNTBodyConstraintManager*> TNTBodyConstraintManager::getConnectedComponents(const ConstraintListConst& constraints) const {
	std::set<TNTBodyConstraintManager*> components;

	// Traverse all contacts and constraints and build initial components
	BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
		const TNTBody* const parent = constraint->getParent();
		const TNTBody* const child = constraint->getChild();

		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<TNTBodyConstraintManager*>::iterator parentPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator childPos = components.end();
		std::set<TNTBodyConstraintManager*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const TNTBody* const b, (*it)->getBodies()) {
				if (parent == b)
					parentPos = it;
				if (child == b)
					childPos = it;
				if (parentPos != components.end() && childPos != components.end())
					break;
			}
		}
		if (parentPos == components.end() && childPos == components.end()) {
			// If not found in any components, a new one is added
			// (we already know that at least one of the bodies must be dynamic)
			TNTBodyConstraintManager* const newComponent = new TNTBodyConstraintManager(this);
			components.insert(newComponent);
			newComponent->addBody(parent);
			newComponent->addBody(child);
			newComponent->addConstraints(getConstraints(parent,child));
		} else if (parentPos == components.end() && childPos != components.end()) {
			// A component was found for one of the bodies, and the other body is added to this component
			(*childPos)->addBody(parent);
			(*childPos)->addConstraints(getConstraints(parent,child));
		} else if (parentPos != components.end() && childPos == components.end()) {
			// A component was found for one of the bodies, and the other body is added to this component
			(*parentPos)->addBody(child);
			(*parentPos)->addConstraints(getConstraints(parent,child));
		} else if (parentPos != components.end() && childPos != components.end()) {
			// A component was found for both bodies (if it is the same component, constraints have been added)
			if (parentPos != childPos) {
				// The two components are different, and are therefore merged into one
				(*parentPos)->addBodies((*childPos)->getBodies());
				(*parentPos)->addConstraints((*childPos)->getPersistentConstraints());
				(*parentPos)->addConstraints(getConstraints(parent,child));
				delete (*childPos);
				components.erase(childPos);
			}
		}
	}

	return components;
}
