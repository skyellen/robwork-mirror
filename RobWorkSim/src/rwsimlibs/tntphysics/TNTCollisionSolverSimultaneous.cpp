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

#include "TNTCollisionSolverSimultaneous.hpp"
#include "TNTRigidBody.hpp"
#include "TNTContact.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTSettings.hpp"
#include "TNTRestitutionModel.hpp"
#include "TNTIslandState.hpp"
#include "TNTMaterialMap.hpp"

#ifdef TNT_DEBUG_ENABLE_BOUNCING
#include <rwsim/dynamics/Body.hpp>
#endif

#include <rwsim/dynamics/RigidBody.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_RESOLVER_TOLERANCE "TNTCollisionSolverResolverTolerance"
#define PROPERTY_SVD_PRECISION "TNTCollisionSolverSingularValuePrecision"

TNTCollisionSolverSimultaneous::TNTCollisionSolverSimultaneous() {
}

TNTCollisionSolverSimultaneous::~TNTCollisionSolverSimultaneous() {
}

void TNTCollisionSolverSimultaneous::doCollisions(
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
		handleComponent(componentContacts, *component, map, tntstate, rwstate, pmap);
		delete component;
	}
}

void TNTCollisionSolverSimultaneous::addDefaultProperties(PropertyMap& map) const {
	TNTCollisionSolver::addDefaultProperties(map);
	addDefaultPropertiesInternal(map);
}

void TNTCollisionSolverSimultaneous::addDefaultPropertiesInternal(PropertyMap& map) {
	map.add<double>(PROPERTY_RESOLVER_TOLERANCE,"Resolver will activate contacts with colliding velocity greater than this, and deactivate contacts that has leaving velocity greater than this (in m/s).",1e-6);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
}

void TNTCollisionSolverSimultaneous::handleComponent(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& component,
	const TNTMaterialMap* const map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap)
{
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
	TNT_DEBUG_BOUNCING("Component has " << constraints.size() << " constraints and contacts.");

	// Extract the contacts
	std::vector<const TNTContact*> contactsComponent;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		if (const TNTContact* contact = dynamic_cast<const TNTContact*>(constraint))
			contactsComponent.push_back(contact);
	}

	// Make resolution loop
	resolveContacts(component.getDynamicBodies(), constraints, map, tntstate, rwstate, pmap);
}

void TNTCollisionSolverSimultaneous::resolveContacts(
	const std::list<const TNTRigidBody*>& component,
	const std::list<const TNTConstraint*>& constraints,
	const TNTMaterialMap* map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap)
{
	// First find the properties to use
	double RESOLVER_THRESHOLD = pmap.get<double>(PROPERTY_RESOLVER_TOLERANCE,-1.);
	double PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);
	if (RESOLVER_THRESHOLD < 0 || PRECISION < 0) {
		PropertyMap tmpMap;
		addDefaultPropertiesInternal(tmpMap);
		if (RESOLVER_THRESHOLD < 0) {
			RESOLVER_THRESHOLD = tmpMap.get<double>(PROPERTY_RESOLVER_TOLERANCE,-1.);
			RW_ASSERT(RESOLVER_THRESHOLD > 0);
		}
		if (PRECISION < 0) {
			PRECISION = tmpMap.get<double>(PROPERTY_SVD_PRECISION,-1.);
			RW_ASSERT(PRECISION > 0);
		}
	}

	TNTIslandState tmpState;

	// Construct initial lists assuming all known contacts will be leaving and only the new contacts will penetrate.
	std::vector<const TNTContact*> candidates;
	std::vector<const TNTConstraint*> nonContacts;
	BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
		RW_ASSERT(constraint != NULL);
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		if (contact)
			candidates.push_back(contact);
		else
			nonContacts.push_back(constraint);
	}
	std::vector<bool> enabled(candidates.size(),false);

	bool repeat = true;
	std::list<std::vector<bool> > testedCombinations;
	std::list<std::vector<bool> > allCombinations;
	bool testedAll = false;
	while (repeat) {
		// Check if we have already tested this combination before.
		bool testedCombination = false;
		BOOST_FOREACH(const std::vector<bool>& comb, testedCombinations) {
			bool match = true;
			for (std::size_t i = 0; i < comb.size(); i++) {
				if (comb[i] != enabled[i]) {
					match = false;
					break;
				}
			}
			if (match) {
				testedCombination = true;
				if (allCombinations.size() > 0) {
					allCombinations.erase(allCombinations.begin());
					if (allCombinations.size() == 0)
						testedAll = true;
				}
				break;
			}
		}
		if (testedAll)
			RW_THROW("TNTCollisionSolverSimultaneous (resolveContacts): all combinations tested - none was valid.");
		// If loop was found, we break it by suggesting a new combination
		if (testedCombination) {
			// Construct list of all possible combinations if not already done
			if (allCombinations.size() == 0) {
				const std::size_t nrOfContacts = candidates.size();
				RW_ASSERT(nrOfContacts > 0);
				if (nrOfContacts >= 24)
					RW_THROW("TNTCollisionSolverSimultaneous (resolveContacts): There are too many contacts (" << nrOfContacts << " of max 24) - please reduce the number of contacts or increase the value of \"" << PROPERTY_RESOLVER_TOLERANCE << "\".");
				std::size_t nrOfComb = 2;
				for (std::size_t i = 1; i < nrOfContacts; i++)
					nrOfComb *= 2;
				allCombinations.resize(nrOfComb);
				std::size_t i = 0;
				BOOST_FOREACH(std::vector<bool>& comb, allCombinations) {
					comb.resize(nrOfContacts);
					std::size_t val = i;
					for (std::size_t k = 0; k < nrOfContacts; k++) {
						comb[k] = val%2;
						val = val >> 1;
					}
					i++;
				}
			}
			// Now try the first combination in list
			enabled = allCombinations.front();
			TNT_DEBUG_BOUNCING("Combination suggested by heuristic already tested - trying a different one (" << allCombinations.size() << " left).");
			continue;
		}
		// Add the current combinations to the list of tested combinations
		testedCombinations.push_back(enabled);

		// Construct list of contacts
		std::vector<const TNTContact*> penetratingContacts;
		for (std::size_t i = 0; i < enabled.size(); i++) {
			if (enabled[i]) {
				penetratingContacts.push_back(candidates[i]);
			}
		}

		// Now try to solve
		tmpState = tntstate;
		Eigen::VectorXd solution;
		if (penetratingContacts.size() > 0 || nonContacts.size() > 0) {
			solution = solve(penetratingContacts, nonContacts, map, tmpState, rwstate, PRECISION);
			applySolution(solution, component, penetratingContacts, nonContacts, map, tmpState, rwstate);
		}
		repeat = false;
		Eigen::MatrixXd::Index solId = 0;
		BOOST_FOREACH(const TNTConstraint* constraint, nonContacts) {
			solId += constraint->getDimVelocity();
		}
		for (std::size_t i = 0; i < enabled.size(); i++) {
			const TNTContact* const contact = candidates[i];
			if (!enabled[i]) {
				const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
				const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
				const Vector3D<> linRelVel = linVelI-linVelJ;
				const Vector3D<> nij = contact->getNormalW(tmpState);
				const bool leaving = dot(-linRelVel,nij) >= -RESOLVER_THRESHOLD;
				if (!leaving) {
					enabled[i] = true;
					repeat = true;
				}
			} else {
				const TNTRestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,tntstate,rwstate);
				bool outwards = true;
				if (!restitution.enableTangent) {
					RW_ASSERT(solId < solution.rows());
					outwards = solution[solId] < RESOLVER_THRESHOLD;
				} else if (restitution.enableTangent) {
					RW_ASSERT(solId+2 < solution.rows());
					outwards = solution[solId+2] < RESOLVER_THRESHOLD;
				}
				if (!outwards) {
					enabled[i] = false;
					repeat = true;
				}
				solId++;
				if (restitution.enableTangent)
					solId += 2;
				if (restitution.enableAngular)
					solId += 1;
			}
		}
		RW_ASSERT(solId == solution.rows());
	}
	BOOST_FOREACH(const TNTRigidBody* const body, component) {
		body->setVelocityW(body->getVelocityW(tmpState),tntstate);
	}
}

Eigen::VectorXd TNTCollisionSolverSimultaneous::solve(
	const std::vector<const TNTContact*>& contacts,
	const std::vector<const TNTConstraint*>& constraints,
	const TNTMaterialMap* map,
	const TNTIslandState& tntstate,
	const State& rwstate,
	double precision)
{
	RW_ASSERT(contacts.size() > 0 || constraints.size() > 0);

	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimVel = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		RW_ASSERT(constraint != NULL);
		dimVel += constraint->getDimVelocity();
	}
	BOOST_FOREACH(const TNTContact* contact, contacts) {
		RW_ASSERT(contact != NULL);
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const bool leaving = dot(-linRelVel,nij) >= 0;

		dimVel += 1;
		if (!leaving) {
			const TNTRestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,tntstate,rwstate);
			if (restitution.enableTangent)
				dimVel += 2;
			if (restitution.enableAngular)
				dimVel += 1;
		}
	}
	Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(dimVel,dimVel);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dimVel);

	dimVel = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const VelocityScrew6D<>& velI = constraint->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(tntstate,rwstate);
		const Rotation3D<> linRot = constraint->getLinearRotationParentW(tntstate);
		const Rotation3D<> angRot = constraint->getAngularRotationParentW(tntstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();

		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		if (dim > 0) {
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			// Construct RHS
			Eigen::VectorXd::Index dimCnt = dimVel;
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					rhs[dimCnt] = -dot(linRelVel,linRot.getCol(i));
					dimCnt++;
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == TNTConstraint::Velocity) {
					rhs[dimCnt] = -dot(angRelVel,angRot.getCol(i));
					dimCnt++;
				}
			}
			dimCnt = dimVel;
			// Construct LHS blocks
			Eigen::MatrixXd::Index dimVelB = 0;
			BOOST_FOREACH(const TNTConstraint* constraintB, constraints) {
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
				if (dimB > 0) {
					lhs.block(dimVel,dimVelB,dim,dimB) = getBlock(constraint,constraintB,tntstate,rwstate);
					dimVelB += dimB;
				}
			}
			BOOST_FOREACH(const TNTContact* contactB, contacts) {
				const TNTRestitutionModel& restitutionModel = map->getRestitutionModel(*contactB);
				const Eigen::MatrixXd block = getBlock(constraint,contactB,restitutionModel,tntstate,rwstate);
				RW_ASSERT(block.rows() == dim);
				const Eigen::MatrixXd::Index dimB = block.cols();
				lhs.block(dimVel,dimVelB,dim,dimB) = block;
				dimVelB += dimB;
			}
			dimVel += dim;
		}
	}
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		Eigen::VectorXd::Index dim = 1;
		// Construct RHS
		if (leaving) {
			rhs[dimVel] = -dot(linRelVel,nij);
		} else {
			const TNTRestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,tntstate,rwstate);
			if (restitution.enableTangent)
				dim += 2;
			if (restitution.enableAngular)
				dim += 1;

			if (!restitution.enableTangent && !restitution.enableAngular) {
				rhs[dimVel] = -(1+restitution.normal)*dot(linRelVel,nij);
			} else if (restitution.enableTangent && !restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.tangent)*dot(linRelVel,tangentDir);
				rhs[dimVel+1] = -dot(linRelVel,zeroDir);
				rhs[dimVel+2] = -(1+restitution.normal)*dot(linRelVel,nij);
			} else if (!restitution.enableTangent && restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.normal)*dot(linRelVel,nij);
				rhs[dimVel+1] = -(1+restitution.angular)*dot(angRelVel,nij);
			} else if (restitution.enableTangent && restitution.enableAngular) {
				rhs[dimVel+0] = -(1+restitution.tangent)*dot(linRelVel,tangentDir);
				rhs[dimVel+1] = -dot(linRelVel,zeroDir);
				rhs[dimVel+2] = -(1+restitution.normal)*dot(linRelVel,nij);
				rhs[dimVel+3] = -(1+restitution.angular)*dot(angRelVel,nij);
			}
		}
		// Construct LHS blocks
		Eigen::MatrixXd::Index dimVelB = 0;
		BOOST_FOREACH(const TNTConstraint* constraintB, constraints) {
			const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
			if (dimB > 0) {
				const TNTRestitutionModel& restitutionModel = map->getRestitutionModel(*contact);
				lhs.block(dimVel,dimVelB,dim,dimB) = getBlock(contact,constraintB,restitutionModel,tntstate,rwstate);
				dimVelB += dimB;
			}
		}
		BOOST_FOREACH(const TNTContact* contactB, contacts) {
			const TNTRestitutionModel& restitutionModelA = map->getRestitutionModel(*contact);
			const TNTRestitutionModel& restitutionModelB = map->getRestitutionModel(*contactB);
			const Eigen::MatrixXd block = getBlock(contact,contactB,restitutionModelA,restitutionModelB,tntstate,rwstate);
			RW_ASSERT(block.rows() == dim);
			const Eigen::MatrixXd::Index dimB = block.cols();
			lhs.block(dimVel,dimVelB,dim,dimB) = block;
			dimVelB += dimB;
		}
		dimVel += dim;
	}
	RW_ASSERT(lhs.rows() == lhs.cols());

	// Solve
	TNT_DEBUG_BOUNCING("Solving " << lhs.rows() << " x " << lhs.cols() << " equation system.");
	const Eigen::VectorXd sol = LinearAlgebra::pseudoInverse(lhs,precision)*rhs;
	TNT_DEBUG_BOUNCING("Residual: " << (rhs-lhs*sol).transpose() << ".");

	return sol;
}


void TNTCollisionSolverSimultaneous::applySolution(
	const Eigen::VectorXd& solution,
	const std::list<const TNTRigidBody*>& component,
	const std::vector<const TNTContact*>& contacts,
	const std::vector<const TNTConstraint*>& constraints,
	const TNTMaterialMap* map,
	TNTIslandState& tntstate,
	const State& rwstate)
{
	RW_ASSERT(contacts.size() > 0 || constraints.size() > 0);
	Eigen::MatrixXd::Index dimVel = 0;
	typedef std::pair<Vector3D<>, Wrench6D<> > Impulse;
	std::vector<std::list<Impulse> > impulses(component.size());
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		const TNTBody* const cParent = constraint->getParent();
		const TNTBody* const cChild = constraint->getChild();
		const TNTRigidBody* const rcParent = dynamic_cast<const TNTRigidBody*>(cParent);
		const TNTRigidBody* const rcChild = dynamic_cast<const TNTRigidBody*>(cChild);

		if (dim > 0) {
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			const Rotation3D<> linRot = constraint->getLinearRotationParentW(tntstate);
			const Rotation3D<> angRot = constraint->getAngularRotationParentW(tntstate);

			Vector3D<> impF = Vector3D<>::zero();
			Vector3D<> impN = Vector3D<>::zero();
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == TNTConstraint::Velocity) {
					impF += solution[dimVel]*linRot.getCol(i);
					dimVel++;
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == TNTConstraint::Velocity) {
					impN += solution[dimVel]*angRot.getCol(i);
					dimVel++;
				}
			}

			const Vector3D<> rij = constraint->getPositionParentW(tntstate);
			const Vector3D<> rji = constraint->getPositionChildW(tntstate);
			std::size_t i = 0;
			BOOST_FOREACH(const TNTRigidBody* const body, component) {
				if (body == rcParent)
					impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(impF,impN)));
				else
					impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(-impF,-impN)));
				if (body == rcChild)
					impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(-impF,-impN)));
				else
					impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(impF,impN)));
				i++;
			}
		}
	}
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		const TNTBody* const cParent = contact->getParent();
		const TNTBody* const cChild = contact->getChild();
		const TNTRigidBody* const rcParent = dynamic_cast<const TNTRigidBody*>(cParent);
		const TNTRigidBody* const rcChild = dynamic_cast<const TNTRigidBody*>(cChild);

		const VelocityScrew6D<>& velI = contact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(tntstate);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		TNTRestitutionModel::Values restitution;
		if (!leaving)
			restitution = map->getRestitutionModel(*contact).getRestitution(*contact,tntstate,rwstate);

		Vector3D<> impF = Vector3D<>::zero();
		Vector3D<> impN = Vector3D<>::zero();
		if (!leaving && restitution.enableTangent) {
			impF += solution[dimVel]*tangentDir;
			dimVel++;
			impF += solution[dimVel]*zeroDir;
			dimVel++;
		}
		impF += solution[dimVel]*nij;
		dimVel++;
		if (!leaving && restitution.enableAngular) {
			impN = solution[dimVel]*nij;
			dimVel++;
		}

		const Vector3D<> rij = contact->getPositionParentW(tntstate);
		const Vector3D<> rji = contact->getPositionChildW(tntstate);
		std::size_t i = 0;
		BOOST_FOREACH(const TNTRigidBody* const body, component) {
			if (body == rcParent)
				impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rij,Wrench6D<>(impF,impN)));
			else if (body == rcChild)
				impulses[i].push_back(std::make_pair<Vector3D<>, Wrench6D<> >(rji,Wrench6D<>(-impF,-impN)));
			i++;
		}
	}
	std::size_t i = 0;
	BOOST_FOREACH(const TNTRigidBody* const body, component) {
		BOOST_FOREACH(const Impulse& impulse, impulses[i]) {
			applyImpulse(impulse.second, impulse.first, *body, tntstate);
		}
		i++;
	}
}


Eigen::MatrixXd TNTCollisionSolverSimultaneous::getBlock(
	const TNTConstraint* constraintA,
	const TNTConstraint* constraintB,
	const TNTIslandState& tntstate,
	const State& rwstate)
{
	const TNTBody* const parentA = constraintA->getParent();
	const TNTBody* const childA = constraintA->getChild();
	const TNTBody* const parentB = constraintB->getParent();
	const TNTBody* const childB = constraintB->getChild();

	const Eigen::VectorXd::Index dimA = constraintA->getDimVelocity();
	const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
	if (parentA != parentB && parentA != childB &&
			childA != parentB && childA != childB)
	{
		return Eigen::MatrixXd::Zero(dimA,dimB);
	}

	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 6x6 matrix in world coordinates
	const Vector3D<> rij = constraintA->getPositionParentW(tntstate);
	const Vector3D<> rji = constraintA->getPositionChildW(tntstate);
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();
	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = constraintB->getPositionParentW(tntstate);
	else
		rik = constraintB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = constraintB->getPositionParentW(tntstate);
	else
		rjk = constraintB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d bNlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d BFang = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d bNang = Eigen::Matrix3d::Zero();
	if (rParentA && (parentA == parentB || parentA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e();
		BFang += sign*inertiaInvI.e()*Math::skew(rik-RI);
		bNang += sign*inertiaInvI.e();
	}
	if (rChildA && (childA == parentB || childA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e();
		BFang += sign*inertiaInvJ.e()*Math::skew(rjk-RJ);
		bNang += sign*inertiaInvJ.e();
	}

	// Now make rotated block of correct size
	const std::vector<TNTConstraint::Mode> modesA = constraintA->getConstraintModes();
	const std::vector<TNTConstraint::Mode> modesB = constraintB->getConstraintModes();
	const Rotation3D<> linRotA = constraintA->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotA = constraintA->getAngularRotationParentW(tntstate);
	const Rotation3D<> linRotB = constraintB->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotB = constraintB->getAngularRotationParentW(tntstate);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	for (std::size_t i = 0; i < 3; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*bNlin*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}
	for (std::size_t i = 3; i < 6; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*BFang*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == TNTConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*bNang*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}

	return block;
}

Eigen::MatrixXd TNTCollisionSolverSimultaneous::getBlock(
	const TNTConstraint* constraintA,
	const TNTContact* contactB,
	const TNTRestitutionModel& restitutionModel,
	const TNTIslandState& tntstate,
	const State& rwstate)
{
	const TNTBody* const parentA = constraintA->getParent();
	const TNTBody* const childA = constraintA->getChild();
	const TNTBody* const parentB = contactB->getParent();
	const TNTBody* const childB = contactB->getChild();

	const VelocityScrew6D<>& velIB = contactB->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJB = contactB->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVelB = velIB.linear()-velJB.linear();
	const Vector3D<> nijB = contactB->getNormalW(tntstate);
	const bool leavingB = dot(-linRelVelB,nijB) >= 0;

	TNTRestitutionModel::Values restitutionB;
	if (!leavingB)
		restitutionB = restitutionModel.getRestitution(*contactB,tntstate,rwstate);

	const Eigen::VectorXd::Index dimA = constraintA->getDimVelocity();
	Eigen::VectorXd::Index dimB = 1;
	if (!leavingB && restitutionB.enableTangent)
		dimB += 2;
	if (!leavingB && restitutionB.enableAngular)
		dimB += 1;

	if (parentA != parentB && parentA != childB &&
			childA != parentB && childA != childB)
	{
		return Eigen::MatrixXd::Zero(dimA,dimB);
	}

	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 6x4 matrix in world coordinates
	const Vector3D<> rij = constraintA->getPositionParentW(tntstate);
	const Vector3D<> rji = constraintA->getPositionChildW(tntstate);
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	const Vector3D<> zeroDirB = normalize(cross(nijB,linRelVelB));
	const Vector3D<> tangentDirB = normalize(cross(zeroDirB,nijB));

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = contactB->getPositionParentW(tntstate);
	else
		rik = contactB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = contactB->getPositionParentW(tntstate);
	else
		rjk = contactB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double,3,1> bNlin = Eigen::Matrix<double,3,1>::Zero();
	Eigen::Matrix3d BFang = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double,3,1> bNang = Eigen::Matrix<double,3,1>::Zero();
	if (rParentA && (parentA == parentB || parentA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e()*nijB.e();
		BFang += sign*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingB && restitutionB.enableAngular)
			bNang += sign*inertiaInvI.e()*nijB.e();
	}
	if (rChildA && (childA == parentB || childA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e()*nijB.e();
		BFang += sign*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingB && restitutionB.enableAngular)
			bNang += sign*inertiaInvJ.e()*nijB.e();
	}

	// Now make rotated block of correct size
	const std::vector<TNTConstraint::Mode> modesA = constraintA->getConstraintModes();
	const Rotation3D<> linRotA = constraintA->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotA = constraintA->getAngularRotationParentW(tntstate);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;

	for (std::size_t i = 0; i < 3; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			if (!leavingB && restitutionB.enableTangent) {
				block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*tangentDirB.e();
				col++;
				block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*zeroDirB.e();
				col++;
			}
			block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*nijB.e();
			col++;
			if (!leavingB && restitutionB.enableAngular) {
				block(row,col) = linRotA.getCol(i).e().transpose()*bNlin;
				col++;
			}
			row++;
		}
	}
	for (std::size_t i = 3; i < 6; i++) {
		if (modesA[i] == TNTConstraint::Velocity) {
			col = 0;
			if (!leavingB && restitutionB.enableTangent) {
				block(row,col) = angRotA.getCol(i-3).e().transpose()*BFang*tangentDirB.e();
				col++;
				block(row,col) = angRotA.getCol(i-3).e().transpose()*BFang*zeroDirB.e();
				col++;
			}
			block(row,col) = angRotA.getCol(i-3).e().transpose()*BFang*nijB.e();
			col++;
			if (!leavingB && restitutionB.enableAngular) {
				block(row,col) = angRotA.getCol(i-3).e().transpose()*bNang;
				col++;
			}
			row++;
		}
	}
	return block;
}

Eigen::MatrixXd TNTCollisionSolverSimultaneous::getBlock(
	const TNTContact* contactA,
	const TNTConstraint* constraintB,
	const TNTRestitutionModel& restitutionModel,
	const TNTIslandState& tntstate,
	const State& rwstate)
{
	const TNTBody* const parentA = contactA->getParent();
	const TNTBody* const childA = contactA->getChild();
	const TNTBody* const parentB = constraintB->getParent();
	const TNTBody* const childB = constraintB->getChild();

	const VelocityScrew6D<>& velI = contactA->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJ = contactA->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVel = velI.linear()-velJ.linear();
	const Vector3D<> nij = contactA->getNormalW(tntstate);
	const bool leavingA = dot(-linRelVel,nij) >= 0;

	TNTRestitutionModel::Values restitutionA;
	if (!leavingA)
		restitutionA = restitutionModel.getRestitution(*contactA,tntstate,rwstate);

	Eigen::VectorXd::Index dimA = 1;
	const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
	if (!leavingA && restitutionA.enableTangent)
		dimA += 2;
	if (!leavingA && restitutionA.enableAngular)
		dimA += 1;

	if (parentA != parentB && parentA != childB &&
			childA != parentB && childA != childB)
	{
		return Eigen::MatrixXd::Zero(dimA,dimB);
	}

	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 4x6 matrix in world coordinates
	const Vector3D<> rij = contactA->getPositionParentW(tntstate);
	const Vector3D<> rji = contactA->getPositionChildW(tntstate);
	const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
	const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = constraintB->getPositionParentW(tntstate);
	else
		rik = constraintB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = constraintB->getPositionParentW(tntstate);
	else
		rjk = constraintB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d bNlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double,1,3> BFang = Eigen::Matrix<double,1,3>::Zero();
	Eigen::Matrix<double,1,3> bNang = Eigen::Matrix<double,1,3>::Zero();
	if (rParentA && (parentA == parentB || parentA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e();
		if (!leavingA && restitutionA.enableAngular) {
			BFang += sign*nij.e().transpose()*inertiaInvI.e()*Math::skew(rik-RI);
			bNang += sign*nij.e().transpose()*inertiaInvI.e();
		}
	}
	if (rChildA && (childA == parentB || childA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e();
		if (!leavingA && restitutionA.enableAngular) {
			BFang += sign*nij.e().transpose()*inertiaInvJ.e()*Math::skew(rjk-RJ);
			bNang += sign*nij.e().transpose()*inertiaInvJ.e();
		}
	}

	// Now make rotated block of correct size
	const std::vector<TNTConstraint::Mode> modesB = constraintB->getConstraintModes();
	const Rotation3D<> linRotB = constraintB->getLinearRotationParentW(tntstate);
	const Rotation3D<> angRotB = constraintB->getAngularRotationParentW(tntstate);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	if (!leavingA && restitutionA.enableTangent) {
		col = 0;
		for (std::size_t j = 0; j < 3; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row+0,col) = tangentDir.e().transpose()*BFlin*linRotB.getCol(j).e();
				block(row+1,col) = zeroDir.e().transpose()*BFlin*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row+0,col) = tangentDir.e().transpose()*bNlin*angRotB.getCol(j-3).e();
				block(row+1,col) = zeroDir.e().transpose()*bNlin*angRotB.getCol(j-3).e();
				col++;
			}
		}
		row += 2;
	}
	col = 0;
	{
		for (std::size_t j = 0; j < 3; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row,col) = nij.e().transpose()*BFlin*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row,col) = nij.e().transpose()*bNlin*angRotB.getCol(j-3).e();
				col++;
			}
		}
		row++;
	}
	col = 0;
	if (!leavingA && restitutionA.enableAngular) {
		for (std::size_t j = 0; j < 3; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row,col) = BFang*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == TNTConstraint::Velocity) {
				block(row,col) = bNang*angRotB.getCol(j-3).e();
				col++;
			}
		}
		row++;
	}

	return block;
}

Eigen::MatrixXd TNTCollisionSolverSimultaneous::getBlock(
	const TNTContact* contactA,
	const TNTContact* contactB,
	const TNTRestitutionModel& restitutionModelA,
	const TNTRestitutionModel& restitutionModelB,
	const TNTIslandState& tntstate,
	const State& rwstate)
{
	const TNTBody* const parentA = contactA->getParent();
	const TNTBody* const childA = contactA->getChild();
	const TNTBody* const parentB = contactB->getParent();
	const TNTBody* const childB = contactB->getChild();

	const VelocityScrew6D<>& velI = contactA->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJ = contactA->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVel = velI.linear()-velJ.linear();
	const Vector3D<> nij = contactA->getNormalW(tntstate);
	const bool leavingA = dot(-linRelVel,nij) >= 0;

	const VelocityScrew6D<>& velIB = contactB->getVelocityParentW(tntstate,rwstate);
	const VelocityScrew6D<>& velJB = contactB->getVelocityChildW(tntstate,rwstate);
	const Vector3D<> linRelVelB = velIB.linear()-velJB.linear();
	const Vector3D<> nijB = contactB->getNormalW(tntstate);
	const bool leavingB = dot(-linRelVelB,nijB) >= 0;

	TNTRestitutionModel::Values restitutionA;
	TNTRestitutionModel::Values restitutionB;
	if (!leavingA)
		restitutionA = restitutionModelA.getRestitution(*contactA,tntstate,rwstate);
	if (!leavingB)
		restitutionB = restitutionModelB.getRestitution(*contactB,tntstate,rwstate);

	Eigen::VectorXd::Index dimA = 1;
	Eigen::VectorXd::Index dimB = 1;
	if (!leavingA && restitutionA.enableTangent)
		dimA += 2;
	if (!leavingB && restitutionB.enableTangent)
		dimB += 2;
	if (!leavingA && restitutionA.enableAngular)
		dimA += 1;
	if (!leavingB && restitutionB.enableAngular)
		dimB += 1;

	if (parentA != parentB && parentA != childB &&
			childA != parentB && childA != childB)
	{
		return Eigen::MatrixXd::Zero(dimA,dimB);
	}

	const TNTRigidBody* const rParentA = dynamic_cast<const TNTRigidBody*>(parentA);
	const TNTRigidBody* const rChildA = dynamic_cast<const TNTRigidBody*>(childA);
	const TNTRigidBody* const rParentB = dynamic_cast<const TNTRigidBody*>(parentB);
	const TNTRigidBody* const rChildB = dynamic_cast<const TNTRigidBody*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 4x4 matrix in world coordinates
	const Vector3D<> rij = contactA->getPositionParentW(tntstate);
	const Vector3D<> rji = contactA->getPositionChildW(tntstate);
	const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
	const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
	const Transform3D<>& wTbI = parentA->getWorldTcom(tntstate);
	const Transform3D<>& wTbJ = childA->getWorldTcom(tntstate);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	const Vector3D<> zeroDirB = normalize(cross(nijB,linRelVelB));
	const Vector3D<> tangentDirB = normalize(cross(zeroDirB,nijB));

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = contactB->getPositionParentW(tntstate);
	else
		rik = contactB->getPositionChildW(tntstate);
	if (childA == parentB)
		rjk = contactB->getPositionParentW(tntstate);
	else
		rjk = contactB->getPositionChildW(tntstate);

	Eigen::Matrix3d BFlin = Eigen::Matrix3d::Zero();
	Eigen::Matrix<double,3,1> bNlin = Eigen::Matrix<double,3,1>::Zero();
	Eigen::Matrix<double,1,3> BFang = Eigen::Matrix<double,1,3>::Zero();
	double bNang = 0;
	if (rParentA && (parentA == parentB || parentA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyI = rParentA->getRigidBody();
		const InertiaMatrix<>& inertiaInvI = wTbI.R()*rwbodyI->getBodyInertiaInv()*inverse(wTbI.R());
		const double massInvI = rwbodyI->getMassInv();
		BFlin += sign*massInvI*Eigen::Matrix3d::Identity()-sign*Math::skew(rij-RI)*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rij-RI)*inertiaInvI.e()*nijB.e();
		if (!leavingA && restitutionA.enableAngular)
			BFang += sign*nij.e().transpose()*inertiaInvI.e()*Math::skew(rik-RI);
		if (!leavingA && !leavingB && restitutionA.enableAngular && restitutionB.enableAngular)
			bNang += sign*nij.e().transpose()*inertiaInvI.e()*nijB.e();
	}
	if (rChildA && (childA == parentB || childA == childB)) {
		const rw::common::Ptr<const RigidBody> rwbodyJ = rChildA->getRigidBody();
		const InertiaMatrix<>& inertiaInvJ = wTbJ.R()*rwbodyJ->getBodyInertiaInv()*inverse(wTbJ.R());
		const double massInvJ = rwbodyJ->getMassInv();
		BFlin += sign*massInvJ*Eigen::Matrix3d::Identity()-sign*Math::skew(rji-RJ)*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingB && restitutionB.enableAngular)
			bNlin += -sign*Math::skew(rji-RJ)*inertiaInvJ.e()*nijB.e();
		if (!leavingA && restitutionA.enableAngular)
			BFang += sign*nij.e().transpose()*inertiaInvJ.e()*Math::skew(rjk-RJ);
		if (!leavingA && !leavingB && restitutionA.enableAngular && restitutionB.enableAngular)
			//bNang += sign*nij.e().transpose()*inertiaInvJ.e()*nij.e();
			bNang += sign*nij.e().transpose()*inertiaInvJ.e()*nijB.e();
	}

	// Now make rotated block of correct size
	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	if (!leavingA && restitutionA.enableTangent) {
		if (!leavingB && restitutionB.enableTangent) {
			block(row+0,col) = tangentDir.e().transpose()*BFlin*tangentDirB.e();
			block(row+1,col) = zeroDir.e().transpose()*BFlin*tangentDirB.e();
			col++;
			block(row+0,col) = tangentDir.e().transpose()*BFlin*zeroDirB.e();
			block(row+1,col) = zeroDir.e().transpose()*BFlin*zeroDirB.e();
			col++;
		}
		block(row+0,col) = tangentDir.e().transpose()*BFlin*nijB.e();
		block(row+1,col) = zeroDir.e().transpose()*BFlin*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row+0,col) = tangentDir.e().transpose()*bNlin;
			block(row+1,col) = zeroDir.e().transpose()*bNlin;
			col++;
		}
		row += 2;
	}
	col = 0;
	{
		if (!leavingB && restitutionB.enableTangent) {
			block(row,col) = nij.e().transpose()*BFlin*tangentDirB.e();
			col++;
			block(row,col) = nij.e().transpose()*BFlin*zeroDirB.e();
			col++;
		}
		block(row,col) = nij.e().transpose()*BFlin*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row,col) = nij.e().transpose()*bNlin;
			col++;
		}
		row++;
	}
	col = 0;
	if (!leavingA && restitutionA.enableAngular) {
		if (!leavingB && restitutionB.enableTangent) {
			block(row,col) = BFang*tangentDirB.e();
			col++;
			block(row,col) = BFang*zeroDirB.e();
			col++;
		}
		block(row,col) = BFang*nijB.e();
		col++;
		if (!leavingB && restitutionB.enableAngular) {
			block(row,col) = bNang;
			col++;
		}
		row++;
	}
	return block;
}
