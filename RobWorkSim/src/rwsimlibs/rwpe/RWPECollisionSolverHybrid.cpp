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
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPECollisionSolverHybrid.hpp"

#include "RWPEBodyDynamic.hpp"
#include "RWPECollisionSolverSimultaneous.hpp"
#include "RWPEContact.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

#define PROPERTY_PROPTHRES_CONTACT "RWPECollisionSolverPropagateThresholdContact"
#define PROPERTY_PROPTHRES_CONSTRAINTLIN "RWPECollisionSolverPropagateThresholdConstraintLinear"
#define PROPERTY_PROPTHRES_CONSTRAINTANG "RWPECollisionSolverPropagateThresholdConstraintAngular"
#define PROPERTY_MAX_ITERATIONS "RWPECollisionSolverMaxIterations"

RWPECollisionSolverHybrid::RWPECollisionSolverHybrid() {
}

RWPECollisionSolverHybrid::~RWPECollisionSolverHybrid() {
}

void RWPECollisionSolverHybrid::doCollisions(
	const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& bc,
	const RWPEMaterialMap* const map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log,
	rw::common::Ptr<ThreadTask> task) const
{
	const bool doLog = (log == NULL)? false : log->doLog();

	// First connected dynamic body components are found.
	const std::set<RWPEBodyConstraintGraph*> components = bc.getDynamicComponents(islandState);
	if (doLog) {
		log->beginSection("Collision Decomposition",RWPE_LOCATION);
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Components");
		labels.push_back("Bouncing Contacts");
		values.push_back(components.size());
		values.push_back(contacts.size());
		log->addValues("Found components",values,labels,RWPE_LOCATION);
		BOOST_FOREACH(const RWPEBodyConstraintGraph* const bc, components) {
			log->addPositions("Component",bc,rwstate,RWPE_LOCATION);
		}
		log->endSection(__LINE__);
	}

	BOOST_FOREACH(const RWPEBodyConstraintGraph* component, components) {
		std::vector<const RWPEContact*> componentContacts;
		BOOST_FOREACH(const RWPEContact* const contact, contacts) {
			bool found = false;
			BOOST_FOREACH(const RWPEConstraint* const constraint, component->getPersistentConstraints()) {
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
			handleComponent(componentContacts, *component, map, islandState, rwstate, pmap, log);
	}
}

void RWPECollisionSolverHybrid::handleComponent(
	const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& component,
	const RWPEMaterialMap* const map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log) const
{
	const bool doLog = (log == NULL)? false : log->doLog();

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
	if (doLog) {
		std::ostream& sstr = log->log("Handling component",RWPE_LOCATION);
		sstr << "Handling component with bodies:";
		BOOST_FOREACH(const RWPEBody* const body, component.getBodies()) {
			sstr << " " << body->get()->getName();
		}
	}

	// Find all constraints in component (including known contacts)
	std::list<const RWPEConstraint*> constraints;
	{
		const RWPEBodyConstraintGraph::ConstraintList list = component.getPersistentConstraints();
		constraints.insert(constraints.begin(),list.begin(),list.end());
	}

	// All initially colliding contacts are are added to set
	std::set<const RWPEConstraint*> colliding;
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		colliding.insert(contact);
	}
	if (doLog) {
		std::ostream& sstr = log->log("Component Stats",RWPE_LOCATION);
		sstr << "Component has " << constraints.size() << " constraints and contacts and " << colliding.size() << " initially colliding contacts.";
		BOOST_FOREACH(const RWPEBody* const body, component.getBodies()) {
			sstr << " " << body->get()->getName();
		}
	}

	// The disabled set makes sure that contacts are not treated as colliding before they can
	// actually be colliding due to the initial contacts (the disabled set is reduced to an
	// empty set as the initial collisions propagate).
	std::set<const RWPEConstraint*> disabled(constraints.begin(),constraints.end());
	BOOST_FOREACH(const RWPEConstraint* const c, colliding) {
		disabled.erase(c);
	}

	// Now run loop
	std::size_t iterations = 0;
	while (colliding.size() > 0 && (int)iterations <= MAX_ITERATIONS) {
		iterations++;
		//RWPE_DEBUG_BOUNCING("Iteration " << iterations);
		// First we try to decompose the colliding set into subcomponents that can be solved independently
		// (and simultaneously)
		std::list<const RWPEConstraint*> collidingList(colliding.begin(),colliding.end());
		//RWPE_DEBUG_BOUNCING("Colliding: " << collidingList.size());
		const std::set<RWPEBodyConstraintGraph*> subcomponents = component.getConnectedComponents(collidingList);
		//RWPE_DEBUG_BOUNCING("Subcomponents found: " << subcomponents.size());
		// Now solve each (remember to destroy afterwards):
		BOOST_FOREACH(const RWPEBodyConstraintGraph* const subcomponent, subcomponents) {
			if (doLog) {
				std::ostream& sstr = log->log("Handling subcomponent",RWPE_LOCATION);
				sstr << "Handling subcomponent with bodies:";
				BOOST_FOREACH(const RWPEBody* const body, subcomponent->getBodies()) {
					sstr << " " << body->get()->getName();
				}
			}
			// The contacts and constraints in use now, are removed from the disabled list
			const RWPEBodyConstraintGraph::ConstraintList scConstraints = subcomponent->getPersistentConstraints();
			if (disabled.size() > 0) {
				BOOST_FOREACH(RWPEConstraint* const scConstraint, scConstraints) {
					disabled.erase(scConstraint);
				}
			}
			// This could be done in parallel!
			RWPEBodyConstraintGraph::ConstraintListConst scConstraintsConst;
			BOOST_FOREACH(const RWPEConstraint* const scConstraint, scConstraints) {
				scConstraintsConst.push_back(scConstraint);
			}
			RWPECollisionSolverSimultaneous::resolveContacts(subcomponent->getDynamicBodies(), scConstraintsConst, map, islandState, rwstate, pmap);
		}
		// Now a new colliding set must be constructed
		colliding.clear();
		BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
			const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
			const VelocityScrew6D<>& velI = constraint->getVelocityParentW(islandState,rwstate);
			const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(islandState,rwstate);
			const Vector3D<> linRelVel = velI.linear()-velJ.linear();
			const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
			bool solvePair = false;
			if (contact) {
				const Vector3D<> nij = contact->getNormalW(islandState);
				solvePair = dot(-linRelVel,nij) < -THRESHOLD_CONTACT;
			} else {
				if (constraint->getDimVelocity() == 0)
					continue;
				const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
				Vector3D<> linRelVelConstraint = Vector3D<>::zero();
				Vector3D<> angRelVelConstraint = Vector3D<>::zero();
				for (std::size_t modeI = 0; modeI < 6; modeI++) {
					if (modes[modeI] == RWPEConstraint::Velocity) {
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
					BOOST_FOREACH(const RWPEBodyConstraintGraph* const subcomponent, subcomponents) {
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
		BOOST_FOREACH(const RWPEBodyConstraintGraph* const subcomponent, subcomponents) {
			delete subcomponent;
		}
		if ((int)iterations == MAX_ITERATIONS)
			RW_THROW("RWPECollisionSolverHybrid (handleComponent): maximum number of iterations reached: " << MAX_ITERATIONS);
	}
}

void RWPECollisionSolverHybrid::addDefaultProperties(PropertyMap& map) const {
	RWPECollisionSolverSimultaneous solver;
	solver.addDefaultProperties(map);
	map.add<double>(PROPERTY_PROPTHRES_CONTACT,"Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).",1e-4);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,"Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).",1e-8);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,"Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).",1e-8);
	map.add<int>(PROPERTY_MAX_ITERATIONS,"If impulses are still propagating after this number of iterations, an exception is thrown.",1000);
}
