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

#include "TNTContactResolverHeuristic.hpp"
#include "TNTSolver.hpp"
#include "TNTContact.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTMaterialMap.hpp"
#include "TNTFrictionModel.hpp"
#include "TNTIslandState.hpp"
#include "TNTRigidBody.hpp"
#include "TNTIntegrator.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_PROPMAXITERATIONS "TNTContactResolverMaxIterations"
#define PROPERTY_PROPMAXVEL "TNTContactResolverMaxPenetrationVelocity"
#define PROPERTY_PROPMAXFORCE "TNTContactResolverMaxAttractionForce"

TNTContactResolverHeuristic::TNTContactResolverHeuristic():
	_solver(NULL)
{
}

TNTContactResolverHeuristic::TNTContactResolverHeuristic(const TNTSolver* const solver):
	_solver(solver)
{
}

TNTContactResolverHeuristic::~TNTContactResolverHeuristic() {
}

const TNTContactResolver* TNTContactResolverHeuristic::createResolver(const TNTSolver* solver) const {
	return new TNTContactResolverHeuristic(solver);
}

void TNTContactResolverHeuristic::solve(const std::vector<TNTContact*>& persistentContacts,
		double h, const TNTMaterialMap& map, const State &rwstate, TNTIslandState &tntstate,
		const PropertyMap& pmap) const
{
	if (_solver == NULL)
		RW_THROW("TNTContactResolverHeuristic (solve): There is no TNTSolver set for this resolver - please construct a new resolver for TNTSolver to use.");
	int maxIterations = pmap.get<int>(PROPERTY_PROPMAXITERATIONS,-1);
	double maxPenetrationVelocity = pmap.get<double>(PROPERTY_PROPMAXVEL,-1);
	double maxAttractionForce = pmap.get<double>(PROPERTY_PROPMAXFORCE,-1);

	if (maxIterations < 0 || maxPenetrationVelocity < 0 || maxAttractionForce < 0) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (maxIterations < 0) {
			maxIterations = tmpMap.get<int>(PROPERTY_PROPMAXITERATIONS,-1);
			RW_ASSERT(maxIterations > 0);
		}
		if (maxPenetrationVelocity < 0) {
			maxPenetrationVelocity = tmpMap.get<double>(PROPERTY_PROPMAXVEL,-1);
			RW_ASSERT(maxPenetrationVelocity >= 0);
		}
		if (maxAttractionForce < 0) {
			maxAttractionForce = tmpMap.get<double>(PROPERTY_PROPMAXFORCE,-1);
			RW_ASSERT(maxAttractionForce >= 0);
		}
	}

	// Construct list of contacts
	std::vector<TNTContact*> contacts;
	{
		const TNTBodyConstraintManager::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&tntstate);
		BOOST_FOREACH(TNTConstraint* const constraint, constraints) {
			if (TNTContact* const contact = dynamic_cast<TNTContact*>(constraint)) {
				contacts.push_back(contact);
			}
		}
	}

	// Construct map of allowed states according to friction models
	std::vector<Type> types(contacts.size(),TanOff_AngOff);
	std::vector<Choice> initialChoice(contacts.size(),Leaving);
	std::size_t i = 0;
	std::size_t combinations = 1;
	BOOST_FOREACH(TNTContact* const contact, contacts) {
		const TNTFrictionModel& model = map.getFrictionModel(*contact->getParent(),*contact->getChild());
		const TNTFrictionModel::Values vals = model.getRestitution(*contact,tntstate,rwstate);
		// Determine which modes are allowed for each contact and save for later use
		// (4 different combinations of tangent and angular on/off)
		if (vals.enableAngular) {
			RW_THROW("TNTContactResolverHeuristic (solve): angular friction is not yet handled!");
		} else if (!vals.enableTangent && !vals.enableAngular) {
			types[i] = TanOff_AngOff;
			combinations *= 2;
		} else if (vals.enableTangent && !vals.enableAngular) {
			types[i] = TanOn_AngOff;
			combinations *= 3;
		} else if (!vals.enableTangent && vals.enableAngular) {
			types[i] = TanOff_AngOn;
			combinations *= 3;
		} else if (vals.enableTangent && vals.enableAngular) {
			types[i] = TanOn_AngOn;
			combinations *= 5;
		}
		// We save the current choice (from last step), which will be the combination we try first.
		// We automatically correct if some contacts have invalid state according to the friction model.
		// Also a sliding contact with a very small relative velocity will be forced into sticking mode first,
		// as no direction can be determined.
		const bool typeLeaving = contact->isLeaving();
		if (typeLeaving) {
			initialChoice[i] = Leaving;
		} else {
			TNTContact::Type typeLinear = contact->getTypeLinear();
			TNTContact::Type typeAngular = contact->getTypeAngular();
			switch(types[i]) {
			case TanOff_AngOff:
				initialChoice[i] = NonLeaving;
				typeLinear = TNTContact::None;
				typeAngular = TNTContact::None;
				break;
			case TanOn_AngOff:
				if (typeLinear == TNTContact::None || typeLinear == TNTContact::Sliding) {
					initialChoice[i] = Sliding;
					typeLinear = TNTContact::Sliding;
					const Vector3D<> velP = contact->getVelocityParentW(tntstate,rwstate).linear();
					const Vector3D<> velC = contact->getVelocityChildW(tntstate,rwstate).linear();
					if ((velC-velP).norm2() < 1e-3) {
						// Relative velocity is too small to determine friction direction - we force it into rolling mode first
						initialChoice[i] = Sticking;
						typeLinear = TNTContact::Sticking;
					}
				} else {
					initialChoice[i] = Sticking;
					typeLinear = TNTContact::Sticking;
				}
				typeAngular = TNTContact::None;
				break;
			case TanOff_AngOn:
				if (typeAngular == TNTContact::None || typeAngular == TNTContact::Sliding) {
					initialChoice[i] = Sliding;
					typeAngular = TNTContact::Sliding;
				} else {
					initialChoice[i] = Sticking;
					typeAngular = TNTContact::Sticking;
				}
				typeLinear = TNTContact::None;
				break;
			case TanOn_AngOn:
				if (typeLinear == TNTContact::None || typeLinear == TNTContact::Sliding) {
					if (typeAngular == TNTContact::None || typeAngular == TNTContact::Sliding) {
						initialChoice[i] = SlidingSliding;
						typeAngular = TNTContact::Sliding;
					} else {
						initialChoice[i] = SlidingSticking;
						typeAngular = TNTContact::Sticking;
					}
					typeLinear = TNTContact::Sliding;
				} else {
					if (typeAngular == TNTContact::None || typeAngular == TNTContact::Sliding) {
						initialChoice[i] = StickingSliding;
						typeAngular = TNTContact::Sliding;
					} else {
						initialChoice[i] = StickingSticking;
						typeAngular = TNTContact::Sticking;
					}
					typeLinear = TNTContact::Sticking;
				}
				break;
			}
			contact->setType(typeLinear, typeAngular);

			// For tangential sliding we update the friction direction with the current direction of motion
			if (typeLinear == TNTContact::Sliding) {
				const Vector3D<> velP = contact->getVelocityParentW(tntstate,rwstate).linear();
				const Vector3D<> velC = contact->getVelocityChildW(tntstate,rwstate).linear();
				contact->setFrictionDirW(normalize(velC-velP),rwstate);
			}
		}
		i++;
	}

	TNTIslandState tmpState;
	TNTIslandState resState;
	std::list<std::vector<Choice> > testedCombinations;
	bool repeat = true;
	unsigned int iterations = 0;
	while (repeat) {
		if ((int)iterations == maxIterations && maxIterations != 0)
			RW_THROW("TNTContactResolverHeuristic (solve): maximum number of iterations (" << maxIterations << ") reached in contact resolution.");

		// Check if we have already tested this combination before.
		bool testedCombination = false;
		BOOST_FOREACH(const std::vector<Choice>& comb, testedCombinations) {
			bool match = true;
			std::size_t i = 0;
			BOOST_FOREACH(const Choice val, comb) {
				const TNTContact* const contact = contacts[i];
				const TNTContact::Type typeLinear = contact->getTypeLinear();
				const TNTContact::Type typeAngular = contact->getTypeAngular();
				if (contact->isLeaving()) {
					if (val != Leaving) {
						match = false;
						break;
					}
				} else {
					switch(types[i]) {
					case TanOff_AngOff:
						if (typeLinear == TNTContact::None && val != NonLeaving)
							match = false;
						break;
					case TanOn_AngOff:
						if (typeLinear == TNTContact::Sliding/* && val != Sliding */)
							match = false;
						else if (typeLinear == TNTContact::Sticking && val != Sticking)
							match = false;
						break;
					case TanOff_AngOn:
						if (typeAngular == TNTContact::Sliding && val != Sliding)
							match = false;
						else if (typeAngular == TNTContact::Sticking && val != Sticking)
							match = false;
						break;
					case TanOn_AngOn:
						if (typeLinear == TNTContact::Sliding && typeAngular == TNTContact::Sliding && val != SlidingSliding)
							match = false;
						else if (typeLinear == TNTContact::Sliding && typeAngular == TNTContact::Sticking && val != SlidingSticking)
							match = false;
						else if (typeLinear == TNTContact::Sticking && typeAngular == TNTContact::Sliding && val != StickingSliding)
							match = false;
						else if (typeLinear == TNTContact::Sticking && typeAngular == TNTContact::Sticking && val != StickingSticking)
							match = false;
						break;
					}
				}
				i++;
			}
			if (match) {
				testedCombination = true;
				break;
			}
		}

		// If loop was found, we break it by suggesting a new combination
		if (testedCombination) {
			RW_THROW("TNTContactResolverHeuristic (solve): loop encountered - can not handle loops yet!");
		}

		// Add the current combination to the list of tested combinations
		testedCombinations.push_back(std::vector<Choice>(contacts.size()));
		std::size_t i = 0;
		BOOST_FOREACH(const TNTContact* const contact, contacts) {
			if (contact->isLeaving()) {
				testedCombinations.back()[i] = Leaving;
 			} else {
				const TNTContact::Type typeLinear = contact->getTypeLinear();
				const TNTContact::Type typeAngular = contact->getTypeAngular();

				if (typeLinear == TNTContact::None && typeAngular == TNTContact::None)
					testedCombinations.back()[i] = NonLeaving;
				else if (typeLinear == TNTContact::None && typeAngular == TNTContact::Sliding)
					testedCombinations.back()[i] = Sliding;
				else if (typeLinear == TNTContact::None && typeAngular == TNTContact::Sticking)
					testedCombinations.back()[i] = Sticking;
				else if (typeLinear == TNTContact::Sliding && typeAngular == TNTContact::None)
					testedCombinations.back()[i] = Sliding;
				else if (typeLinear == TNTContact::Sticking && typeAngular == TNTContact::None)
					testedCombinations.back()[i] = Sticking;
				else if (typeLinear == TNTContact::Sliding && typeAngular == TNTContact::Sliding)
					testedCombinations.back()[i] = SlidingSliding;
				else if (typeLinear == TNTContact::Sliding && typeAngular == TNTContact::Sticking)
					testedCombinations.back()[i] = SlidingSticking;
				else if (typeLinear == TNTContact::Sticking && typeAngular == TNTContact::Sliding)
					testedCombinations.back()[i] = StickingSliding;
				else if (typeLinear == TNTContact::Sticking && typeAngular == TNTContact::Sticking)
					testedCombinations.back()[i] = StickingSticking;
				else
					RW_THROW("TNTContactResolverHeuristic (solve): encountered unknown contact mode.");
			}
			i++;
		}

		// Now try to solve
		const Eigen::VectorXd solution = _solver->solve(h, rwstate, tntstate, pmap);
		tmpState = tntstate;
		_solver->saveSolution(solution,tmpState);
		resState = tmpState;

		// Update velocities (but keep the same positions)
		{
			const TNTBodyConstraintManager::DynamicBodyList rbodies = _solver->getManager()->getDynamicBodies();
			BOOST_FOREACH(const TNTRigidBody* rbody, rbodies) {
				TNTRigidBody::RigidConfiguration* config = dynamic_cast<TNTRigidBody::RigidConfiguration*>(tmpState.getConfiguration(rbody));
				const Transform3D<> wTcom = config->getWorldTcom();
				rbody->getIntegrator()->integrate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config,tmpState,rwstate);
				config->setWorldTcom(wTcom);
			}
		}
		// Now check if solutions are valid
		repeat = false;
		i = 0;
		BOOST_FOREACH(TNTContact* contact, contacts) {
			const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
			const Vector3D<> linRelVel = linVelI-linVelJ;
			const Vector3D<> nij = contact->getNormalW(tmpState);
			if (contact->isLeaving()) {
				const bool penetrating = dot(-linRelVel,nij) < -maxPenetrationVelocity;
				if (penetrating) {
					repeat = true;
					switch(types[i]) {
					case TanOff_AngOff:
						contact->setType(TNTContact::None,TNTContact::None);
						break;
					case TanOn_AngOff:
						contact->setType(TNTContact::Sliding,TNTContact::None);
						break;
					case TanOff_AngOn:
						contact->setType(TNTContact::None,TNTContact::Sliding);
						break;
					case TanOn_AngOn:
						contact->setType(TNTContact::Sliding,TNTContact::Sliding);
						break;
					}
				}
			} else {
				const Wrench6D<> wrench = contact->getWrenchConstraint(tmpState);
				if (dot(wrench.force(),nij) > -maxAttractionForce) {
					repeat = true;
					contact->setTypeLeaving();
				} else {
					const TNTContact::Type typeLinear = contact->getTypeLinear();
					//const TNTContact::Type typeAngular = contact->getTypeAngular();
					switch(types[i]) {
					case TanOff_AngOff:
						// Already handled
						break;
					case TanOn_AngOff:
					{
						const TNTFrictionModel& model = map.getFrictionModel(*contact->getParent(),*contact->getChild());
						const TNTFrictionModel::Values vals = model.getRestitution(*contact,tntstate,rwstate);
						if (typeLinear == TNTContact::Sliding) {
							const Vector3D<> velP = contact->getVelocityParentW(tmpState,rwstate).linear();
							const Vector3D<> velC = contact->getVelocityChildW(tmpState,rwstate).linear();
							const Vector3D<> velRelDir = normalize(velC-velP);
							const Wrench6D<> wrench = contact->getWrench(tmpState);
							const Vector3D<> normal = contact->getNormalW(tntstate);
							const Vector3D<> F_tangent = wrench.force()-dot(normal,wrench.force())*normal;
							if (dot(velRelDir,F_tangent) <= -1e-15) {
								// Tangent force must be in the direction the child moves relative to its parent
								repeat = true;
								contact->setType(TNTContact::Sticking,TNTContact::None);
							}
						} else if (typeLinear == TNTContact::Sticking) {
							const Wrench6D<> wrench = contact->getWrench(tmpState);
							const Vector3D<> normal = contact->getNormalW(tntstate);
							const double F_normal = dot(normal,wrench.force());
							const Vector3D<> F_tangent = wrench.force()-dot(normal,wrench.force())*normal;
							if (F_tangent.norm2() > fabs(vals.tangent*F_normal)) {
								repeat = true;
								contact->setType(TNTContact::Sliding,TNTContact::None);
								contact->setFrictionDirW(normalize(F_tangent),rwstate);
								contact->setFriction(vals.tangent,vals.tangentAbsolute,vals.angular,vals.angularAbsolute);
							}
						} else {
							RW_THROW("TNTContactResolverHeuristic (solve): encountered invalid contact mode.");
						}
					}
					break;
					case TanOff_AngOn:
						RW_THROW("TNTContactResolverHeuristic (solve): can not yet handle angular friction.");
						break;
					case TanOn_AngOn:
						RW_THROW("TNTContactResolverHeuristic (solve): can not yet handle angular friction.");
						break;
					}
				}
			}
			i++;
		}

		iterations++;
	}

	// Contacts that are leaving should be removed at this point
	BOOST_FOREACH(TNTContact* contact, contacts) {
		if (contact->isLeaving())
			_solver->getManager()->removeTemporaryConstraint(contact,resState);
	}

	tntstate = resState;
}

void TNTContactResolverHeuristic::addDefaultProperties(PropertyMap& map) const {
	map.add<int>(PROPERTY_PROPMAXITERATIONS,"Stop if resolver exceeds this number of iterations (use 0 to test all combinations).",1000);
	map.add<double>(PROPERTY_PROPMAXVEL,"Continue resolving as long as there are contacts with penetrating relative velocities greater than this (m/s).",1e-5);
	map.add<double>(PROPERTY_PROPMAXFORCE,"Continue resolving as long as there are contacts with attracting forces greater than this (N).",0);
}
