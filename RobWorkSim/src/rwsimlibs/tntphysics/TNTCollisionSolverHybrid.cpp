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

#include "TNTCollisionSolverHybrid.hpp"
#include "TNTCollisionSolverSimultaneous.hpp"
#include "TNTRigidBody.hpp"
#include "TNTContact.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTSettings.hpp"

#ifdef TNT_DEBUG_ENABLE_BOUNCING
#include <rwsim/dynamics/Body.hpp>
#endif

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_PROPTHRES_CONTACT "TNTCollisionSolverPropagateThresholdContact"
#define PROPERTY_PROPTHRES_CONSTRAINTLIN "TNTCollisionSolverPropagateThresholdConstraintLinear"
#define PROPERTY_PROPTHRES_CONSTRAINTANG "TNTCollisionSolverPropagateThresholdConstraintAngular"
#define PROPERTY_MAX_ITERATIONS "TNTCollisionSolverMaxIterations"

TNTCollisionSolverHybrid::TNTCollisionSolverHybrid() {
}

TNTCollisionSolverHybrid::~TNTCollisionSolverHybrid() {
}

void TNTCollisionSolverHybrid::doCollisions(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& bc,
	const TNTMaterialMap* const map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap,
	rw::common::Ptr<ThreadTask> task) const
{
	// First connected dynamic body components are found.
	const std::set<TNTBodyConstraintManager*> components = bc.getDynamicComponents(tntstate);
	if (components.size() > 0) {
		TNT_DEBUG_BOUNCING("Found " << components.size() << " component(s) for the " << contacts.size() << " bouncing contact(s).");
	} else {
		TNT_DEBUG_BOUNCING("Found no component for the " << contacts.size() << " bouncing contact(s).");
	}

	BOOST_FOREACH(const TNTBodyConstraintManager* component, components) {
		std::vector<const TNTContact*> componentContacts;
		BOOST_FOREACH(const TNTContact* const contact, contacts) {
			bool found = false;
			BOOST_FOREACH(const TNTConstraint* const constraint, component->getPersistentConstraints()) {
				if (constraint == contact) {
					found = true;
					break;
				}
			}
			if (found) {
				componentContacts.push_back(contact);
			}
		}
		if (componentContacts.size() > 0)
			handleComponent(componentContacts, *component, map, tntstate, rwstate, pmap);
	}
}

void TNTCollisionSolverHybrid::handleComponent(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& component,
	const TNTMaterialMap* const map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const rw::common::PropertyMap& pmap) const
{
	// First find the properties to use
	double THRESHOLD_CONTACT = pmap.get<int>(PROPERTY_PROPTHRES_CONTACT,-1);
	double THRESHOLD_CONSTRAINT_LIN = pmap.get<int>(PROPERTY_PROPTHRES_CONSTRAINTLIN,-1);
	double THRESHOLD_CONSTRAINT_ANG = pmap.get<int>(PROPERTY_PROPTHRES_CONSTRAINTANG,-1);
	int MAX_ITERATIONS = pmap.get<int>(PROPERTY_MAX_ITERATIONS,-1);
	if (MAX_ITERATIONS < 0 || THRESHOLD_CONTACT < 0 || THRESHOLD_CONSTRAINT_LIN < 0 || THRESHOLD_CONSTRAINT_ANG < 0) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (MAX_ITERATIONS < 0) {
			MAX_ITERATIONS = tmpMap.get<int>(PROPERTY_MAX_ITERATIONS,-1);
			RW_ASSERT(MAX_ITERATIONS > 0);
		}
		if (THRESHOLD_CONTACT < 0) {
			THRESHOLD_CONTACT = tmpMap.get<double>(PROPERTY_PROPTHRES_CONTACT,-1);
			RW_ASSERT(THRESHOLD_CONTACT > 0);
		}
		if (THRESHOLD_CONSTRAINT_LIN < 0) {
			THRESHOLD_CONSTRAINT_LIN = tmpMap.get<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,-1);
			RW_ASSERT(THRESHOLD_CONSTRAINT_LIN > 0);
		}
		if (THRESHOLD_CONSTRAINT_ANG < 0) {
			THRESHOLD_CONSTRAINT_ANG = tmpMap.get<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,-1);
			RW_ASSERT(THRESHOLD_CONSTRAINT_ANG > 0);
		}
	}

	// Now handle each component
#ifdef TNT_DEBUG_ENABLE_BOUNCING
	std::stringstream sstr;
	sstr << "Handling component with bodies:";
	BOOST_FOREACH(const TNTBody* const body, component.getBodies()) {
		sstr << " " << body->get()->getName();
	}
	TNT_DEBUG_BOUNCING(sstr.str());
#endif
	// Find all constraints in component (including known contacts)
	std::list<const TNTConstraint*> constraints;
	{
		const TNTBodyConstraintManager::ConstraintList list = component.getPersistentConstraints();
		constraints.insert(constraints.begin(),list.begin(),list.end());
	}

	// All initially colliding contacts are are added to set
	std::set<const TNTConstraint*> colliding;
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		colliding.insert(contact);
	}
	TNT_DEBUG_BOUNCING("Component has " << constraints.size() << " constraints and contacts and " << colliding.size() << " initially colliding contacts.");

	// The disabled set makes sure that contacts are not treated as colliding before they can
	// actually be colliding due to the initial contacts (the disabled set is reduced to an
	// empty set as the initial collisions propagate).
	std::set<const TNTConstraint*> disabled(constraints.begin(),constraints.end());
	BOOST_FOREACH(const TNTConstraint* const c, colliding) {
		disabled.erase(c);
	}

	// Now run loop
	std::size_t iterations = 0;
	while (colliding.size() > 0 && (int)iterations <= MAX_ITERATIONS) {
		iterations++;
		TNT_DEBUG_BOUNCING("Iteration " << iterations);
		// First we try to decompose the colliding set into subcomponents that can be solved independently
		// (and simultaneously)
		std::list<const TNTConstraint*> collidingList(colliding.begin(),colliding.end());
		TNT_DEBUG_BOUNCING("Colliding: " << collidingList.size());
		const std::set<TNTBodyConstraintManager*> subcomponents = component.getConnectedComponents(collidingList);
		TNT_DEBUG_BOUNCING("Subcomponents found: " << subcomponents.size());
		// Now solve each (remember to destroy afterwards):
		BOOST_FOREACH(const TNTBodyConstraintManager* const subcomponent, subcomponents) {
#ifdef TNT_DEBUG_ENABLE_BOUNCING
			std::stringstream sstr;
			sstr << "Handling subcomponent with bodies:";
			BOOST_FOREACH(const TNTBody* const body, subcomponent->getBodies()) {
				sstr << " " << body->get()->getName();
			}
			TNT_DEBUG_BOUNCING(sstr.str());
#endif
			// The contacts and constraints in use now, are removed from the disabled list
			const TNTBodyConstraintManager::ConstraintList scConstraints = subcomponent->getPersistentConstraints();
			if (disabled.size() > 0) {
				BOOST_FOREACH(TNTConstraint* const scConstraint, scConstraints) {
					disabled.erase(scConstraint);
				}
			}
			// This could be done in parallel!
			TNTBodyConstraintManager::ConstraintListConst scConstraintsConst;
			BOOST_FOREACH(const TNTConstraint* const scConstraint, scConstraints) {
				scConstraintsConst.push_back(scConstraint);
			}
			TNTCollisionSolverSimultaneous::resolveContacts(subcomponent->getDynamicBodies(), scConstraintsConst, map, tntstate, rwstate, pmap);
		}
		// Now a new colliding set must be constructed
		colliding.clear();
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
			const VelocityScrew6D<>& velI = constraint->getVelocityParentW(tntstate,rwstate);
			const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(tntstate,rwstate);
			const Vector3D<> linRelVel = velI.linear()-velJ.linear();
			const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
			bool solvePair = false;
			if (contact) {
				const Vector3D<> nij = contact->getNormalW(tntstate);
				solvePair = dot(-linRelVel,nij) < -THRESHOLD_CONTACT;
			} else {
				if (constraint->getDimVelocity() == 0)
					continue;
				const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
				Vector3D<> linRelVelConstraint = Vector3D<>::zero();
				Vector3D<> angRelVelConstraint = Vector3D<>::zero();
				for (std::size_t modeI = 0; modeI < 6; modeI++) {
					if (modes[modeI] == TNTConstraint::Velocity) {
						if (modeI < 3) {
							const Vector3D<> dir = constraint->getLinearRotationParent().getCol(modeI);
							linRelVelConstraint += dot(dir,linRelVel)*dir;
						} else {
							const Vector3D<> dir = constraint->getAngularRotationParent().getCol(modeI-3);
							angRelVelConstraint += dot(dir,angRelVel)*dir;
						}
					}
				}
				if (linRelVelConstraint.normInf() > THRESHOLD_CONSTRAINT_LIN)
					solvePair = true;
				else if (angRelVelConstraint.normInf() > THRESHOLD_CONSTRAINT_ANG)
					solvePair = true;
			}
			if (solvePair) {
				// Search after all new colliding (if a constraint is colliding, add it if not in disable set)
				if (disabled.count(constraint) == 0)
					colliding.insert(constraint);
				else {
					// If in disable set: check if it is connected to the existing colliding list.
					bool found = false;
					BOOST_FOREACH(const TNTBodyConstraintManager* const subcomponent, subcomponents) {
						if (subcomponent->has(constraint->getParent()) || subcomponent->has(constraint->getChild())) {
							found = true;
							break;
						}
					}
					if (found) {
						colliding.insert(constraint);
						disabled.erase(constraint);
					}
				}
			}
		}
		// Cleanup
		BOOST_FOREACH(const TNTBodyConstraintManager* const subcomponent, subcomponents) {
			delete subcomponent;
		}
		if ((int)iterations == MAX_ITERATIONS)
			RW_THROW("TNTCollisionSolverHybrid (handleComponent): maximum number of iterations reached: " << MAX_ITERATIONS);
	}
}

void TNTCollisionSolverHybrid::addDefaultProperties(PropertyMap& map) const {
	TNTCollisionSolverSimultaneous solver;
	solver.addDefaultProperties(map);
	map.add<double>(PROPERTY_PROPTHRES_CONTACT,"Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).",1e-4);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,"Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).",1e-8);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,"Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).",1e-8);
	map.add<int>(PROPERTY_MAX_ITERATIONS,"If impulses are still propagating after this number of iterations, an exception is thrown.",1000);
}
