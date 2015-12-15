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

#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEConstraintSolver.hpp"
#include "RWPEContact.hpp"
#include "RWPEContactResolverHeuristic.hpp"
#include "RWPEFrictionModel.hpp"
#include "RWPEIntegrator.hpp"
#include "RWPEIslandState.hpp"
#include "RWPEMaterialMap.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

#define PROPERTY_PROPMAXITERATIONS "RWPEContactResolverMaxIterations"
#define PROPERTY_PROPMAXVEL "RWPEContactResolverMaxPenetrationVelocity"
#define PROPERTY_PROPMAXFORCE "RWPEContactResolverMaxAttractionForce"

RWPEContactResolverHeuristic::RWPEContactResolverHeuristic():
	_solver(NULL)
{
}

RWPEContactResolverHeuristic::RWPEContactResolverHeuristic(const RWPEConstraintSolver* const solver):
	_solver(solver)
{
}

RWPEContactResolverHeuristic::~RWPEContactResolverHeuristic() {
}

const RWPEContactResolver* RWPEContactResolverHeuristic::createResolver(const RWPEConstraintSolver* solver) const {
	return new RWPEContactResolverHeuristic(solver);
}

void RWPEContactResolverHeuristic::solve(const std::vector<RWPEContact*>& persistentContacts,
	double h, bool discontinuity, const RWPEMaterialMap& map, const State &rwstate,
	const RWPEIslandState &islandState0, RWPEIslandState &islandStateH,
	const PropertyMap& pmap,
	class RWPELogUtil* log) const
{
	if (_solver == NULL)
		RW_THROW("RWPEContactResolverHeuristic (solve): There is no RWPEConstraintSolver set for this resolver - please construct a new resolver for RWPEConstraintSolver to use.");
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
	std::vector<RWPEContact*> contacts;
	{
		const RWPEBodyConstraintGraph::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&islandStateH);
		BOOST_FOREACH(RWPEConstraint* const constraint, constraints) {
			if (RWPEContact* const contact = dynamic_cast<RWPEContact*>(constraint)) {
				contacts.push_back(contact);
			}
		}
	}

	// Construct map of allowed states according to friction models
	std::vector<Type> types(contacts.size(),TanOff_AngOff);
	std::vector<Choice> initialChoice(contacts.size(),Leaving);
	std::size_t i = 0;
	std::size_t combinations = 1;
	BOOST_FOREACH(RWPEContact* const contact, contacts) {
		const RWPEFrictionModel& model = map.getFrictionModel(*contact->getParent(),*contact->getChild());
		const RWPEFrictionModel::Values vals = model.getFriction(*contact,islandStateH,rwstate);
		// Determine which modes are allowed for each contact and save for later use
		// (4 different combinations of tangent and angular on/off)
		if (vals.enableAngular) {
			RW_THROW("RWPEContactResolverHeuristic (solve): angular friction is not yet handled!");
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
			RWPEContact::Type typeLinear = contact->getTypeLinear();
			RWPEContact::Type typeAngular = contact->getTypeAngular();
			switch(types[i]) {
			case TanOff_AngOff:
				initialChoice[i] = NonLeaving;
				typeLinear = RWPEContact::None;
				typeAngular = RWPEContact::None;
				break;
			case TanOn_AngOff:
				std::cout << "Type: " << RWPEContact::toString(typeLinear) << std::endl;
				if (typeLinear == RWPEContact::None || typeLinear == RWPEContact::Sliding) {
					std::cout << "A" << std::endl;
					initialChoice[i] = Sliding;
					typeLinear = RWPEContact::Sliding;
					const Vector3D<> velP = contact->getVelocityParentW(islandStateH,rwstate).linear();
					const Vector3D<> velC = contact->getVelocityChildW(islandStateH,rwstate).linear();
					if ((velC-velP).norm2() < 1e-3) {
						// Relative velocity is too small to determine friction direction - we force it into rolling mode first
						initialChoice[i] = Sticking;
						typeLinear = RWPEContact::Sticking;
					}
				} else {
					std::cout << "B" << std::endl;
					initialChoice[i] = Sticking;
					typeLinear = RWPEContact::Sticking;
				}
				typeAngular = RWPEContact::None;
				break;
			case TanOff_AngOn:
				if (typeAngular == RWPEContact::None || typeAngular == RWPEContact::Sliding) {
					initialChoice[i] = Sliding;
					typeAngular = RWPEContact::Sliding;
				} else {
					initialChoice[i] = Sticking;
					typeAngular = RWPEContact::Sticking;
				}
				typeLinear = RWPEContact::None;
				break;
			case TanOn_AngOn:
				if (typeLinear == RWPEContact::None || typeLinear == RWPEContact::Sliding) {
					if (typeAngular == RWPEContact::None || typeAngular == RWPEContact::Sliding) {
						initialChoice[i] = SlidingSliding;
						typeAngular = RWPEContact::Sliding;
					} else {
						initialChoice[i] = SlidingSticking;
						typeAngular = RWPEContact::Sticking;
					}
					typeLinear = RWPEContact::Sliding;
				} else {
					if (typeAngular == RWPEContact::None || typeAngular == RWPEContact::Sliding) {
						initialChoice[i] = StickingSliding;
						typeAngular = RWPEContact::Sliding;
					} else {
						initialChoice[i] = StickingSticking;
						typeAngular = RWPEContact::Sticking;
					}
					typeLinear = RWPEContact::Sticking;
				}
				break;
			}
			contact->setType(typeLinear, typeAngular);

			// For tangential sliding we update the friction direction with the current direction of motion
			if (typeLinear == RWPEContact::Sliding) {
				const Vector3D<> velP = contact->getVelocityParentW(islandStateH,rwstate).linear();
				const Vector3D<> velC = contact->getVelocityChildW(islandStateH,rwstate).linear();
				contact->setFrictionDirW(normalize(velC-velP),rwstate);
				contact->setFriction(vals.tangent,vals.tangentAbsolute,vals.angular,vals.angularAbsolute);
			}
		}
		i++;
	}

	RWPEIslandState tmpState;
	RWPEIslandState resState;
	std::list<std::vector<Choice> > testedCombinations;
	bool repeat = true;
	unsigned int iterations = 0;
	while (repeat) {
		if ((int)iterations == maxIterations && maxIterations != 0)
			RW_THROW("RWPEContactResolverHeuristic (solve): maximum number of iterations (" << maxIterations << ") reached in contact resolution.");

		// Check if we have already tested this combination before.
		bool testedCombination = false;
		BOOST_FOREACH(const std::vector<Choice>& comb, testedCombinations) {
			bool match = true;
			std::size_t i = 0;
			BOOST_FOREACH(const Choice val, comb) {
				const RWPEContact* const contact = contacts[i];
				const RWPEContact::Type typeLinear = contact->getTypeLinear();
				const RWPEContact::Type typeAngular = contact->getTypeAngular();
				if (contact->isLeaving()) {
					if (val != Leaving) {
						match = false;
						break;
					}
				} else {
					switch(types[i]) {
					case TanOff_AngOff:
						if (typeLinear == RWPEContact::None && val != NonLeaving)
							match = false;
						break;
					case TanOn_AngOff:
						if (typeLinear == RWPEContact::Sliding/* && val != Sliding */)
							match = false;
						else if (typeLinear == RWPEContact::Sticking && val != Sticking)
							match = false;
						break;
					case TanOff_AngOn:
						if (typeAngular == RWPEContact::Sliding && val != Sliding)
							match = false;
						else if (typeAngular == RWPEContact::Sticking && val != Sticking)
							match = false;
						break;
					case TanOn_AngOn:
						if (typeLinear == RWPEContact::Sliding && typeAngular == RWPEContact::Sliding && val != SlidingSliding)
							match = false;
						else if (typeLinear == RWPEContact::Sliding && typeAngular == RWPEContact::Sticking && val != SlidingSticking)
							match = false;
						else if (typeLinear == RWPEContact::Sticking && typeAngular == RWPEContact::Sliding && val != StickingSliding)
							match = false;
						else if (typeLinear == RWPEContact::Sticking && typeAngular == RWPEContact::Sticking && val != StickingSticking)
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
			RW_THROW("RWPEContactResolverHeuristic (solve): loop encountered - can not handle loops yet!");
		}

		// Add the current combination to the list of tested combinations
		testedCombinations.push_back(std::vector<Choice>(contacts.size()));
		std::size_t i = 0;
		BOOST_FOREACH(const RWPEContact* const contact, contacts) {
			if (contact->isLeaving()) {
				testedCombinations.back()[i] = Leaving;
 			} else {
				const RWPEContact::Type typeLinear = contact->getTypeLinear();
				const RWPEContact::Type typeAngular = contact->getTypeAngular();

				if (typeLinear == RWPEContact::None && typeAngular == RWPEContact::None)
					testedCombinations.back()[i] = NonLeaving;
				else if (typeLinear == RWPEContact::None && typeAngular == RWPEContact::Sliding)
					testedCombinations.back()[i] = Sliding;
				else if (typeLinear == RWPEContact::None && typeAngular == RWPEContact::Sticking)
					testedCombinations.back()[i] = Sticking;
				else if (typeLinear == RWPEContact::Sliding && typeAngular == RWPEContact::None)
					testedCombinations.back()[i] = Sliding;
				else if (typeLinear == RWPEContact::Sticking && typeAngular == RWPEContact::None)
					testedCombinations.back()[i] = Sticking;
				else if (typeLinear == RWPEContact::Sliding && typeAngular == RWPEContact::Sliding)
					testedCombinations.back()[i] = SlidingSliding;
				else if (typeLinear == RWPEContact::Sliding && typeAngular == RWPEContact::Sticking)
					testedCombinations.back()[i] = SlidingSticking;
				else if (typeLinear == RWPEContact::Sticking && typeAngular == RWPEContact::Sliding)
					testedCombinations.back()[i] = StickingSliding;
				else if (typeLinear == RWPEContact::Sticking && typeAngular == RWPEContact::Sticking)
					testedCombinations.back()[i] = StickingSticking;
				else
					RW_THROW("RWPEContactResolverHeuristic (solve): encountered unknown contact mode.");
			}
			i++;
		}

		// Now try to solve
		const Eigen::VectorXd solution = _solver->solve(h, discontinuity, rwstate, islandState0, islandStateH, pmap);
		tmpState = islandStateH;
		_solver->saveSolution(solution,tmpState);
		resState = tmpState;

		// Update velocities (but keep the same positions)
		{
			const RWPEBodyConstraintGraph::DynamicBodyList rbodies = _solver->getManager()->getDynamicBodies();
			BOOST_FOREACH(const RWPEBodyDynamic* rbody, rbodies) {
				const RWPEBodyDynamic::RigidConfiguration* const config0 = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(islandState0.getConfiguration(rbody));
				RWPEBodyDynamic::RigidConfiguration* const config = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(tmpState.getConfiguration(rbody));
				//const Transform3D<> wTcom = config->getWorldTcom();
				//rbody->getIntegrator()->integrate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config,tmpState,rwstate);
				//config->setWorldTcom(wTcom);
				rbody->getIntegrator()->velocityUpdate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config0,*config,islandState0,tmpState,rwstate);
			}
		}
		// Now check if solutions are valid
		repeat = false;
		i = 0;
		BOOST_FOREACH(RWPEContact* contact, contacts) {
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
						contact->setType(RWPEContact::None,RWPEContact::None);
						break;
					case TanOn_AngOff:
						contact->setType(RWPEContact::Sliding,RWPEContact::None);
						break;
					case TanOff_AngOn:
						contact->setType(RWPEContact::None,RWPEContact::Sliding);
						break;
					case TanOn_AngOn:
						contact->setType(RWPEContact::Sliding,RWPEContact::Sliding);
						break;
					}
				}
			} else {
				const Wrench6D<> wrench = contact->getWrenchConstraint(tmpState);
				if (dot(wrench.force(),nij) > -maxAttractionForce) {
					repeat = true;
					contact->setTypeLeaving();
				} else {
					const RWPEContact::Type typeLinear = contact->getTypeLinear();
					//const RWPEContact::Type typeAngular = contact->getTypeAngular();
					switch(types[i]) {
					case TanOff_AngOff:
						// Already handled
						break;
					case TanOn_AngOff:
					{
						const RWPEFrictionModel& model = map.getFrictionModel(*contact->getParent(),*contact->getChild());
						const RWPEFrictionModel::Values vals = model.getFriction(*contact,islandStateH,rwstate);
						if (typeLinear == RWPEContact::Sliding) {
							const Vector3D<> velP = contact->getVelocityParentW(tmpState,rwstate).linear();
							const Vector3D<> velC = contact->getVelocityChildW(tmpState,rwstate).linear();
							const Vector3D<> velRelDir = normalize(velC-velP);
							const Wrench6D<> wrench = contact->getWrench(tmpState);
							const Vector3D<> normal = contact->getNormalW(islandStateH);
							const Vector3D<> F_tangent = wrench.force()-dot(normal,wrench.force())*normal;
							if (dot(velRelDir,F_tangent) <= -1e-15) {
								// Tangent force must be in the direction the child moves relative to its parent
								repeat = true;
								contact->setType(RWPEContact::Sticking,RWPEContact::None);
							}
						} else if (typeLinear == RWPEContact::Sticking) {
							const Wrench6D<> wrench = contact->getWrench(tmpState);
							const Vector3D<> normal = contact->getNormalW(islandStateH);
							const double F_normal = dot(normal,wrench.force());
							const Vector3D<> F_tangent = wrench.force()-dot(normal,wrench.force())*normal;
							if (F_tangent.norm2() > fabs(vals.tangent*F_normal)) {
								repeat = true;
								contact->setType(RWPEContact::Sliding,RWPEContact::None);
								contact->setFrictionDirW(normalize(F_tangent),rwstate);
								contact->setFriction(vals.tangent,vals.tangentAbsolute,vals.angular,vals.angularAbsolute);
							}
						} else {
							RW_THROW("RWPEContactResolverHeuristic (solve): encountered invalid contact mode.");
						}
					}
					break;
					case TanOff_AngOn:
						RW_THROW("RWPEContactResolverHeuristic (solve): can not yet handle angular friction.");
						break;
					case TanOn_AngOn:
						RW_THROW("RWPEContactResolverHeuristic (solve): can not yet handle angular friction.");
						break;
					}
				}
			}
			i++;
		}

		iterations++;
	}

	// Contacts that are leaving should be removed at this point
	BOOST_FOREACH(RWPEContact* contact, contacts) {
		if (contact->isLeaving())
			_solver->getManager()->removeTemporaryConstraint(contact,resState);
	}

	islandStateH = resState;
}

void RWPEContactResolverHeuristic::addDefaultProperties(PropertyMap& map) const {
	map.add<int>(PROPERTY_PROPMAXITERATIONS,"Stop if resolver exceeds this number of iterations (use 0 to test all combinations).",1000);
	map.add<double>(PROPERTY_PROPMAXVEL,"Continue resolving as long as there are contacts with penetrating relative velocities greater than this (m/s).",1e-5);
	map.add<double>(PROPERTY_PROPMAXFORCE,"Continue resolving as long as there are contacts with attracting forces greater than this (N).",0);
}
