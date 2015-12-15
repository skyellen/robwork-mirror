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

#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include "RWPEBody.hpp"
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyFixed.hpp"
#include "RWPEBodyKinematic.hpp"
#include "RWPEConstraint.hpp"
#include "RWPEConstraintGeneric.hpp"
#include "RWPEIslandState.hpp"

using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEBodyConstraintGraph::RWPEBodyConstraintGraph():
	_parent(NULL)
{
}

RWPEBodyConstraintGraph::RWPEBodyConstraintGraph(const RWPEBodyConstraintGraph* parent):
	_parent(parent)
{
}

RWPEBodyConstraintGraph::~RWPEBodyConstraintGraph() {
	if (_parent == NULL) {
		BOOST_FOREACH(const RWPEBody* const body, _allBodies) {
			delete body;
		}
		BOOST_FOREACH(const RWPEConstraint* const constraint, _constraints) {
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

void RWPEBodyConstraintGraph::initFromDWC(const rw::common::Ptr<const DynamicWorkCell>& dwc) {
	std::map<rwsim::dynamics::Body*, const RWPEBody*> rwBodyToBody;
	const std::vector<Body::Ptr> bodies = dwc->getBodies();
	BOOST_FOREACH(Body::Ptr body, bodies) {
		const RWPEBody* rwpebody;
		if(RigidBody::Ptr rbody = body.cast<RigidBody>() )
			rwpebody = new RWPEBodyDynamic(rbody);
		else if(FixedBody::Ptr fbody = body.cast<FixedBody>())
			rwpebody = new RWPEBodyFixed(fbody);
		else if(KinematicBody::Ptr kbody = body.cast<KinematicBody>())
			rwpebody = new RWPEBodyKinematic(kbody);
		else
			RW_THROW("BodyContactManager (initFromDWC): The type for body \"" << body->getName() << "\" is unsupported!");
		addBody(rwpebody);
		rwBodyToBody[body.get()] = rwpebody;
	}
	const std::vector<Constraint::Ptr> constraints = dwc->getConstraints();
	BOOST_FOREACH(Constraint::Ptr constraint, constraints) {
		const RWPEBody* const parent = rwBodyToBody[constraint->getBody1()];
		const RWPEBody* const child = rwBodyToBody[constraint->getBody2()];
		if (parent == NULL)
			RW_THROW("BodyContactManager (initFromDWC): The body body \"" << parent->get()->getName() << "\" is unknown for constraint " << constraint->getName() << "!");
		if (child == NULL)
			RW_THROW("BodyContactManager (initFromDWC): The body body \"" << child->get()->getName() << "\" is unknown for constraint " << constraint->getName() << "!");
		RWPERWConstraint* const rwpeconstraint = new RWPERWConstraint(constraint,parent,child);
		addConstraint(rwpeconstraint);
	}
}

RWPEBodyConstraintGraph::BodyList RWPEBodyConstraintGraph::getBodies() const {
	return _allBodies;
}

RWPEBodyConstraintGraph::DynamicBodyList RWPEBodyConstraintGraph::getDynamicBodies() const {
	return _dynamicBodies;
}

RWPEBodyConstraintGraph::KinematicBodyList RWPEBodyConstraintGraph::getKinematicBodies() const {
	return _kinematicBodies;
}

RWPEBodyConstraintGraph::ConstraintList RWPEBodyConstraintGraph::getPersistentConstraints() const {
	return _constraints;
}

RWPEBodyConstraintGraph::ConstraintList RWPEBodyConstraintGraph::getConstraints(const RWPEIslandState& state) const {
	RWPEBodyConstraintGraph::ConstraintList list = _constraints;
	if (_parent == NULL) {
		const RWPEBodyConstraintGraph::ConstraintList templist = state.getTemporaryConstraints();
		list.insert(list.end(),templist.begin(),templist.end());
	}
	return list;
}

RWPEBodyConstraintGraph::ConstraintListConst RWPEBodyConstraintGraph::getConstraints(const RWPEBody* body, const RWPEIslandState& state) const {
	RWPEBodyConstraintGraph::ConstraintListConst list;
	{
		std::map<const RWPEBody*, ConstraintListConst>::const_iterator it = _bodyToConstraints.find(body);
		if (it != _bodyToConstraints.end())
			list = (*it).second;
	}
	if (_parent == NULL) {
		const RWPEBodyConstraintGraph::ConstraintListConst templist = state.getTemporaryConstraints(body);
		list.insert(list.end(),templist.begin(),templist.end());
	}
	return list;
}

RWPEBodyConstraintGraph::ConstraintList RWPEBodyConstraintGraph::getConstraints(const RWPEBody* bodyA, const RWPEBody* bodyB) const {
	RWPEBodyConstraintGraph::ConstraintList list;
	BOOST_FOREACH(RWPEConstraint* constraint, _constraints) {
		if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
				(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
			list.push_back(constraint);
		}
	}
	return list;
}

RWPEBodyConstraintGraph::ConstraintListConst RWPEBodyConstraintGraph::getConstraints(const RWPEBody* bodyA, const RWPEBody* bodyB, const RWPEIslandState& state) const {
	RWPEBodyConstraintGraph::ConstraintListConst list;
	{
		std::map<const RWPEBody*, ConstraintListConst>::const_iterator it = _bodyToConstraints.find(bodyA);
		if (it != _bodyToConstraints.end()) {
			const ConstraintListConst constraintsA = it->second;
			BOOST_FOREACH(const RWPEConstraint* constraint, constraintsA) {
				if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
						(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
					list.push_back(constraint);
				}
			}
		}
	}
	if (_parent == NULL) {
		const RWPEBodyConstraintGraph::ConstraintListConst templist = state.getTemporaryConstraints(bodyA);
		BOOST_FOREACH(const RWPEConstraint* constraint, templist) {
			if ((bodyA == constraint->getParent() && bodyB == constraint->getChild()) ||
					(bodyA == constraint->getChild() && bodyB == constraint->getParent())) {
				list.push_back(constraint);
			}
		}
	}
	return list;
}

RWPEBodyConstraintGraph::ConstraintList RWPEBodyConstraintGraph::getTemporaryConstraints(const RWPEIslandState* state) const {
	if (_parent == NULL)
		return state->getTemporaryConstraints();
	else
		return ConstraintList();
}

bool RWPEBodyConstraintGraph::hasContactsOrConstraints(const RWPEIslandState& state) const {
	if (_constraints.size() > 0)
		return true;
	if (_parent == NULL)
		return state.hasTemporaryConstraints();
	return false;
}

void RWPEBodyConstraintGraph::addBody(const RWPEBody* const body) {
	_allBodies.push_back(body);

	if (const RWPEBodyDynamic* rbody = dynamic_cast<const RWPEBodyDynamic*>(body))
		_dynamicBodies.push_back(rbody);
	else if (const RWPEBodyKinematic* kbody = dynamic_cast<const RWPEBodyKinematic*>(body))
		_kinematicBodies.push_back(kbody);

	_frameToBody[body->get()->getBodyFrame()] = body;
}

void RWPEBodyConstraintGraph::addBodies(const BodyList& bodies) {
	BOOST_FOREACH(const RWPEBody* const body, bodies) {
		addBody(body);
	}
}

void RWPEBodyConstraintGraph::addConstraint(RWPEConstraint* constraint) {
	_constraints.push_back(constraint);
	RW_ASSERT(constraint->getParent() != NULL);
	RW_ASSERT(constraint->getChild() != NULL);
	_bodyToConstraints[constraint->getParent()].push_back(constraint);
	_bodyToConstraints[constraint->getChild()].push_back(constraint);
}

void RWPEBodyConstraintGraph::addConstraints(const ConstraintList& constraints) {
	_constraints.insert(_constraints.end(),constraints.begin(),constraints.end());
	BOOST_FOREACH(RWPEConstraint* const constraint, constraints) {
		RW_ASSERT(constraint->getParent() != NULL);
		RW_ASSERT(constraint->getChild() != NULL);
		_bodyToConstraints[constraint->getParent()].push_back(constraint);
		_bodyToConstraints[constraint->getChild()].push_back(constraint);
	}
}

void RWPEBodyConstraintGraph::addTemporaryConstraint(RWPEConstraint* constraint, RWPEIslandState& state) const {
	if (_parent == NULL) {
		state.addTemporaryConstraint(constraint);
	} else {
		RW_THROW("RWPEBodyConstraintGraph (addTemporaryConstraint): as this manager represents a subcomponent, it is not allowed to add temporary constraints!");
	}
}

void RWPEBodyConstraintGraph::removeBody(const RWPEBody* const body) {
	_allBodies.remove(body);

	if (const RWPEBodyDynamic* rbody = dynamic_cast<const RWPEBodyDynamic*>(body))
		_dynamicBodies.remove(rbody);
	else if (const RWPEBodyKinematic* kbody = dynamic_cast<const RWPEBodyKinematic*>(body))
		_kinematicBodies.remove(kbody);
}

void RWPEBodyConstraintGraph::removeConstraint(RWPEConstraint* constraint) {
	_constraints.remove(constraint);
	_bodyToConstraints[constraint->getParent()].remove(constraint);
	_bodyToConstraints[constraint->getChild()].remove(constraint);
}

void RWPEBodyConstraintGraph::removeTemporaryConstraint(const RWPEConstraint* constraint, RWPEIslandState& state) const {
	if (_parent == NULL) {
		state.removeTemporaryConstraint(constraint);
	} else {
		RW_THROW("RWPEBodyConstraintGraph (removeTemporaryConstraint): as this manager represents a subcomponent, it is not allowed to remove temporary constraints!");
	}
}

void RWPEBodyConstraintGraph::clearTemporaryConstraints(RWPEIslandState& state) const {
	if (_parent == NULL) {
		state.clearTemporaryConstraints();
	} else {
		RW_THROW("RWPEBodyConstraintGraph (clearTemporaryConstraints): as this manager represents a subcomponent, it is not allowed to remove temporary constraints!");
	}
}

const RWPEBody* RWPEBodyConstraintGraph::getBody(const rw::kinematics::Frame* frame) const {
	std::map<const rw::kinematics::Frame*, const RWPEBody*>::const_iterator it = _frameToBody.find(frame);
	if (it != _frameToBody.end())
		return (*it).second;
	return NULL;
}

const RWPEBody* RWPEBodyConstraintGraph::getBody(const std::string& name) const {
	std::map<const rw::kinematics::Frame*, const RWPEBody*>::const_iterator it;
	for (it = _frameToBody.begin(); it != _frameToBody.end(); it++) {
		if (it->first->getName() == name)
			return it->second;
	}
	return NULL;
}

bool RWPEBodyConstraintGraph::has(const RWPEBody* body) const {
	BOOST_FOREACH(const RWPEBody* const b, _allBodies) {
		if (b == body)
			return true;
	}
	return false;
}

std::set<RWPEBodyConstraintGraph*> RWPEBodyConstraintGraph::getDynamicComponents(const RWPEIslandState& state) const {
	std::set<RWPEBodyConstraintGraph*> components;

	// Traverse all contacts and constraints and build initial components
	BOOST_FOREACH(RWPEConstraint* const constraint, getConstraints(state)) {
		if (constraint->getDimVelocity() == 0)
			continue;
		const RWPEBodyDynamic* const parent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const RWPEBodyDynamic* const child = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<RWPEBodyConstraintGraph*>::iterator parentPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator childPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const RWPEBodyDynamic* const b, (*it)->getDynamicBodies()) {
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
			RWPEBodyConstraintGraph* const newComponent = new RWPEBodyConstraintGraph(this);
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
	BOOST_FOREACH(RWPEBodyConstraintGraph* const component, components) {
		BOOST_FOREACH(RWPEConstraint* const constraint, component->getPersistentConstraints()) {
			const RWPEBody* const parent = dynamic_cast<const RWPEBody*>(constraint->getParent());
			const RWPEBody* const child = dynamic_cast<const RWPEBody*>(constraint->getChild());
			const RWPEBody* staticBody = NULL;
			if (!dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent())) {
				staticBody = parent;
			}
			if (!dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild())) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const RWPEBody* const existingBody, component->getBodies()) {
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

std::set<RWPEBodyConstraintGraph*> RWPEBodyConstraintGraph::getDynamicComponents(const std::list<std::pair<const RWPEBody*, const RWPEBody*> >& pairs) const {
	std::set<RWPEBodyConstraintGraph*> components;

	// Traverse all constraints and build initial components
	BOOST_FOREACH(RWPEConstraint* const constraint, getPersistentConstraints()) {
		//if (constraint->getDimVelocity() == 0)
		//	continue;
		const RWPEBodyDynamic* const parent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const RWPEBodyDynamic* const child = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<RWPEBodyConstraintGraph*>::iterator parentPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator childPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const RWPEBodyDynamic* const b, (*it)->getDynamicBodies()) {
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
			RWPEBodyConstraintGraph* const newComponent = new RWPEBodyConstraintGraph(this);
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
	typedef std::pair<const RWPEBody*, const RWPEBody*> BodyPair;
	BOOST_FOREACH(const BodyPair& pair, pairs) {
		const RWPEBodyDynamic* const parent = dynamic_cast<const RWPEBodyDynamic*>(pair.first);
		const RWPEBodyDynamic* const child = dynamic_cast<const RWPEBodyDynamic*>(pair.second);
		if (!parent && !child) {
			continue;
		}
		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<RWPEBodyConstraintGraph*>::iterator parentPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator childPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const RWPEBodyDynamic* const b, (*it)->getDynamicBodies()) {
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
			RWPEBodyConstraintGraph* const newComponent = new RWPEBodyConstraintGraph(this);
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
	BOOST_FOREACH(RWPEBodyConstraintGraph* const component, components) {
		BOOST_FOREACH(RWPEConstraint* const constraint, component->getPersistentConstraints()) {
			const RWPEBody* const parent = constraint->getParent();
			const RWPEBody* const child = constraint->getChild();
			const RWPEBody* staticBody = NULL;
			if (!dynamic_cast<const RWPEBodyDynamic*>(parent)) {
				staticBody = parent;
			}
			if (!dynamic_cast<const RWPEBodyDynamic*>(child)) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const RWPEBody* const existingBody, component->getBodies()) {
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
			const RWPEBody* const parent = pair.first;
			const RWPEBody* const child = pair.second;
			const RWPEBody* staticBody = NULL;
			if (!dynamic_cast<const RWPEBodyDynamic*>(parent)) {
				staticBody = parent;
			}
			if (!dynamic_cast<const RWPEBodyDynamic*>(child)) {
				staticBody = child;
			}
			if (staticBody == NULL)
				continue;
			bool found = false;
			BOOST_FOREACH(const RWPEBody* const existingBody, component->getBodies()) {
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

std::set<RWPEBodyConstraintGraph*> RWPEBodyConstraintGraph::getConnectedComponents(const ConstraintListConst& constraints) const {
	std::set<RWPEBodyConstraintGraph*> components;

	// Traverse all contacts and constraints and build initial components
	BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
		const RWPEBody* const parent = constraint->getParent();
		const RWPEBody* const child = constraint->getChild();

		// Search for the parent and child bodies in the existing components
		//  - if not found they are either static or not yet added
		std::set<RWPEBodyConstraintGraph*>::iterator parentPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator childPos = components.end();
		std::set<RWPEBodyConstraintGraph*>::iterator it;
		for (it = components.begin(); it != components.end(); it++) {
			BOOST_FOREACH(const RWPEBody* const b, (*it)->getBodies()) {
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
			RWPEBodyConstraintGraph* const newComponent = new RWPEBodyConstraintGraph(this);
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
