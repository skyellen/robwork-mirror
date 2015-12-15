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
#include <rwsim/dynamics/RigidBody.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPECollisionSolverSimultaneous.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEContact.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"
#include "RWPEMaterialMap.hpp"
#include "RWPERestitutionModel.hpp"
#include "RWPELinearOptimizer.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

#define PROPERTY_RESOLVER_TOLERANCE "RWPECollisionSolverResolverTolerance"
#define PROPERTY_SVD_PRECISION "RWPECollisionSolverSingularValuePrecision"
#define PROPERTY_MAX_CONTACTS "RWPECollisionSolverMaxContacts"
#define PROPERTY_ITERATIONS "RWPECollisionSolverIterations"

RWPECollisionSolverSimultaneous::RWPECollisionSolverSimultaneous() {
}

RWPECollisionSolverSimultaneous::~RWPECollisionSolverSimultaneous() {
}

void RWPECollisionSolverSimultaneous::doCollisions(
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
		/*std::vector<const RWPEContact*> componentContacts;
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
		}*/
		bool found = false;
		BOOST_FOREACH(const RWPEConstraint* const constraint, component->getPersistentConstraints()) {
			BOOST_FOREACH(const RWPEContact* const contact, contacts) {
				if (constraint == contact) {
					found = true;
					break;
				}
			}
			if (found)
				break;
		}
		if (found)
			handleComponent(/*componentContacts, */ *component, map, islandState, rwstate, pmap, log);
		delete component;
	}
}

void RWPECollisionSolverSimultaneous::addDefaultProperties(PropertyMap& map) const {
	RWPECollisionSolver::addDefaultProperties(map);
	addDefaultPropertiesInternal(map);
}

void RWPECollisionSolverSimultaneous::addDefaultPropertiesInternal(PropertyMap& map) {
	map.add<double>(PROPERTY_RESOLVER_TOLERANCE,"Resolver will activate contacts with colliding velocity greater than this, and deactivate contacts that has leaving velocity greater than this (in m/s).",1e-6);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
	map.add<int>(PROPERTY_MAX_CONTACTS,"Maximum number of simultaneous contacts before throwing exception (only used if solve iterative i disabled).",24);
	map.add<int>(PROPERTY_ITERATIONS,"Maximum number of iterations in iterative solver (set to zero to disable iterative solver).",2500);
}

void RWPECollisionSolverSimultaneous::handleComponent(
	//const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& component,
	const RWPEMaterialMap* const map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log)
{
	const bool doLog = (log == NULL)? false : log->doLog();

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
	if (doLog) {
		std::ostream& sstr = log->log("Component Stats",RWPE_LOCATION);
		sstr << "Component has " << constraints.size() << " constraints.";
		BOOST_FOREACH(const RWPEBody* const body, component.getBodies()) {
			sstr << " " << body->get()->getName();
		}
	}

	// Extract the contacts
	//std::vector<const RWPEContact*> contactsComponent;
	//BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
	//	if (const RWPEContact* contact = dynamic_cast<const RWPEContact*>(constraint))
	//		contactsComponent.push_back(contact);
	//}

	// Make resolution loop
	resolveContacts(component.getDynamicBodies(), constraints, map, islandState, rwstate, pmap, log);
}

void RWPECollisionSolverSimultaneous::resolveContacts(
	const std::list<const RWPEBodyDynamic*>& component,
	const std::list<const RWPEConstraint*>& constraints,
	const RWPEMaterialMap* map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log)
{
	const bool doLog = (log == NULL)? false : log->doLog();

	// First find the properties to use
	double RESOLVER_THRESHOLD = pmap.get<double>(PROPERTY_RESOLVER_TOLERANCE,-1.);
	double PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);
	int MAX_CONTACTS = pmap.get<int>(PROPERTY_MAX_CONTACTS,-1.);
	int ITERATIONS = pmap.get<int>(PROPERTY_ITERATIONS,-1);
	if (RESOLVER_THRESHOLD < 0 || PRECISION < 0 || MAX_CONTACTS < 0 || ITERATIONS < 0) {
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
		if (MAX_CONTACTS < 0) {
			MAX_CONTACTS = tmpMap.get<int>(PROPERTY_MAX_CONTACTS,-1.);
		}
		if (ITERATIONS < 0) {
			ITERATIONS = tmpMap.get<int>(PROPERTY_ITERATIONS,-1.);
			RW_ASSERT(ITERATIONS >= 0);
		}
	}

	// Construct initial lists assuming all known contacts will be leaving and only the new contacts will penetrate.
	std::vector<const RWPEContact*> candidates;
	std::vector<const RWPEConstraint*> nonContacts;
	BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
		RW_ASSERT(constraint != NULL);
		const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
		if (contact)
			candidates.push_back(contact);
		else
			nonContacts.push_back(constraint);
	}

	if (ITERATIONS > 0) {
		if (doLog)
			log->log("Using Iterative Method",RWPE_LOCATION);
		// Now try to solve
		Eigen::VectorXd solution;
		if (candidates.size() > 0 || nonContacts.size() > 0) {
			solution = solve(candidates, nonContacts, map, islandState, rwstate, PRECISION, true, ITERATIONS, log);
			applySolution(solution, component, candidates, nonContacts, map, islandState, rwstate);
		}
		BOOST_FOREACH(const RWPEBodyDynamic* const body, component) {
			const VelocityScrew6D<> vel = body->getVelocityW(islandState);
			body->setVelocityW(vel,islandState);
		}
	} else {
		if (doLog)
			log->log("Using LCP Method",RWPE_LOCATION);
		RWPEIslandState tmpState;
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
				RW_THROW("RWPECollisionSolverSimultaneous (resolveContacts): all combinations tested - none was valid.");
			// If loop was found, we break it by suggesting a new combination
			if (testedCombination) {
				// Construct list of all possible combinations if not already done
				if (allCombinations.size() == 0) {
					const std::size_t nrOfContacts = candidates.size();
					RW_ASSERT(nrOfContacts > 0);
					if ((int)nrOfContacts > MAX_CONTACTS && MAX_CONTACTS > 0)
						RW_THROW("RWPECollisionSolverSimultaneous (resolveContacts): There are too many contacts (" << nrOfContacts << " of max " << MAX_CONTACTS << ") - please reduce the number of contacts or increase the value of \"" << PROPERTY_MAX_CONTACTS << "\".");
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
				if (doLog)
					log->log("Combination suggested by heuristic already tested",RWPE_LOCATION) << " - trying a different one (" << allCombinations.size() << " left).";
				continue;
			}
			// Add the current combinations to the list of tested combinations
			testedCombinations.push_back(enabled);

			// Construct list of contacts
			std::vector<const RWPEContact*> penetratingContacts;
			for (std::size_t i = 0; i < enabled.size(); i++) {
				if (enabled[i]) {
					penetratingContacts.push_back(candidates[i]);
				}
			}

			// Now try to solve
			tmpState = islandState;
			Eigen::VectorXd solution;
			if (penetratingContacts.size() > 0 || nonContacts.size() > 0) {
				solution = solve(penetratingContacts, nonContacts, map, tmpState, rwstate, PRECISION, false, 0);
				applySolution(solution, component, penetratingContacts, nonContacts, map, tmpState, rwstate);
			}
			repeat = false;
			Eigen::MatrixXd::Index solId = 0;
			BOOST_FOREACH(const RWPEConstraint* constraint, nonContacts) {
				solId += constraint->getDimVelocity();
			}
			for (std::size_t i = 0; i < enabled.size(); i++) {
				const RWPEContact* const contact = candidates[i];
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
					const RWPERestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,islandState,rwstate);
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
		BOOST_FOREACH(const RWPEBodyDynamic* const body, component) {
			const VelocityScrew6D<> vel = body->getVelocityW(tmpState);
			body->setVelocityW(vel,islandState);
		}
	}
}

Eigen::VectorXd RWPECollisionSolverSimultaneous::solve(
	const std::vector<const RWPEContact*>& contacts,
	const std::vector<const RWPEConstraint*>& constraints,
	const RWPEMaterialMap* map,
	const RWPEIslandState& islandState,
	const State& rwstate,
	double precision,
	bool iterative,
	unsigned int iterations,
	RWPELogUtil* log)
{
	const bool doLog = (log == NULL)? false : log->doLog();
	RW_ASSERT(contacts.size() > 0 || constraints.size() > 0);

	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimVel = 0;
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		RW_ASSERT(constraint != NULL);
		dimVel += constraint->getDimVelocity();
	}
	const Eigen::MatrixXd::Index dimVelConstraint = dimVel;
	BOOST_FOREACH(const RWPEContact* contact, contacts) {
		RW_ASSERT(contact != NULL);
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(islandState,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(islandState,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(islandState);
		const bool leaving = dot(-linRelVel,nij) >= 0;

		dimVel += 1;
		if (!leaving) {
			const RWPERestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,islandState,rwstate);
			if (restitution.enableTangent)
				dimVel += 2;
			if (restitution.enableAngular)
				dimVel += 1;
		}
	}
	Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(dimVel,dimVel);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dimVel);

	dimVel = 0;
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		const VelocityScrew6D<>& velI = constraint->getVelocityParentW(islandState,rwstate);
		const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(islandState,rwstate);
		const Rotation3D<> linRot = constraint->getLinearRotationParentW(islandState);
		const Rotation3D<> angRot = constraint->getAngularRotationParentW(islandState);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();

		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		if (dim > 0) {
			const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
			// Construct RHS
			Eigen::VectorXd::Index dimCnt = dimVel;
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
					rhs[dimCnt] = -dot(linRelVel,linRot.getCol(i));
					dimCnt++;
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == RWPEConstraint::Velocity) {
					rhs[dimCnt] = -dot(angRelVel,angRot.getCol(i));
					dimCnt++;
				}
			}
			dimCnt = dimVel;
			// Construct LHS blocks
			Eigen::MatrixXd::Index dimVelB = 0;
			BOOST_FOREACH(const RWPEConstraint* constraintB, constraints) {
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
				if (dimB > 0) {
					lhs.block(dimVel,dimVelB,dim,dimB) = getBlock(constraint,constraintB,islandState,rwstate);
					dimVelB += dimB;
				}
			}
			BOOST_FOREACH(const RWPEContact* contactB, contacts) {
				const RWPERestitutionModel& restitutionModel = map->getRestitutionModel(*contactB);
				const Eigen::MatrixXd block = getBlock(constraint,contactB,restitutionModel,islandState,rwstate);
				RW_ASSERT(block.rows() == dim);
				const Eigen::MatrixXd::Index dimB = block.cols();
				lhs.block(dimVel,dimVelB,dim,dimB) = block;
				dimVelB += dimB;
			}
			dimVel += dim;
		}
	}
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		const VelocityScrew6D<>& velI = contact->getVelocityParentW(islandState,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(islandState,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
		const Vector3D<> nij = contact->getNormalW(islandState);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		Eigen::VectorXd::Index dim = 1;
		// Construct RHS
		if (leaving) {
			rhs[dimVel] = -dot(linRelVel,nij);
		} else {
			const RWPERestitutionModel::Values restitution = map->getRestitutionModel(*contact).getRestitution(*contact,islandState,rwstate);
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
		BOOST_FOREACH(const RWPEConstraint* constraintB, constraints) {
			const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
			if (dimB > 0) {
				const RWPERestitutionModel& restitutionModel = map->getRestitutionModel(*contact);
				lhs.block(dimVel,dimVelB,dim,dimB) = getBlock(contact,constraintB,restitutionModel,islandState,rwstate);
				dimVelB += dimB;
			}
		}
		BOOST_FOREACH(const RWPEContact* contactB, contacts) {
			const RWPERestitutionModel& restitutionModelA = map->getRestitutionModel(*contact);
			const RWPERestitutionModel& restitutionModelB = map->getRestitutionModel(*contactB);
			const Eigen::MatrixXd block = getBlock(contact,contactB,restitutionModelA,restitutionModelB,islandState,rwstate);
			RW_ASSERT(block.rows() == dim);
			const Eigen::MatrixXd::Index dimB = block.cols();
			lhs.block(dimVel,dimVelB,dim,dimB) = block;
			dimVelB += dimB;
		}
		dimVel += dim;
	}
	RW_ASSERT(lhs.rows() == lhs.cols());

	// Solve
	if (doLog) {
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Collision Solver Rows");
		labels.push_back("Collision Solver Columns");
		values.push_back(lhs.rows());
		values.push_back(lhs.cols());
		log->addValues("Dimensions",values,labels,RWPE_LOCATION);
	}
	Eigen::VectorXd sol;
	if (iterative) {
		if (doLog)
			log->log("IterativeSVD",RWPE_LOCATION);
		static const double gammaMax = 100;
		static const double ALPHA = 0.01;
		static const double ALPHA_THRESHOLD = 1e-5;
		const RWPELinearOptimizer optimizer(ALPHA,1,1,ALPHA/(ALPHA+1),gammaMax,ALPHA_THRESHOLD,precision);
		sol = optimizer.optimize(lhs,rhs,dimVelConstraint,iterations,precision,log);
		//sol = iterativeSVD(lhs,rhs,dimVelConstraint,iterations,precision,precision,log);
	} else {
		if (doLog)
			log->log("SVD",RWPE_LOCATION);
		sol = LinearAlgebra::pseudoInverse(lhs,precision)*rhs;
	}
	if (doLog)
		log->log("Residual",RWPE_LOCATION) << (rhs-lhs*sol);

	return sol;
}

Eigen::VectorXd RWPECollisionSolverSimultaneous::iterativeSVD(
	const Eigen::MatrixXd& A,
	const Eigen::VectorXd& b,
	Eigen::MatrixXd::Index constraintDim,
	unsigned int iterations,
	double svdPrecision,
	double eps,
	RWPELogUtil* log)
{
	RW_THROW("Deprecated - use RWPELinearOptimizer instead!");
	static const double gammaMax = 100;

	const bool doLog = (log == NULL)? false : log->doLog();
	const Eigen::MatrixXd::Index N = A.rows();

	if (doLog) {
		log->log("Equation System",RWPE_LOCATION) << A << "\n\n" << b;
	}

	// Do a full SVD
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	const double tolerance = svdPrecision * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs().maxCoeff();
	const Eigen::MatrixXd& U = svd.matrixU();
	const Eigen::ArrayXd Wdiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array(), 0);
	//const Eigen::ArrayXd WinvDiag = (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0);
	const int K = (Wdiag != 0).count(); // the rank
	//const Eigen::DiagonalWrapper<const Eigen::VectorXd> W = Eigen::VectorXd(Wdiag).asDiagonal();
	const Eigen::MatrixXd& V = svd.matrixV();

	if (K == N) {
		// Full rank - note we can not be sure that all elements of sX becomes negative
		const Eigen::VectorXd WsDiag(Wdiag);
		const Eigen::DiagonalWrapper<const Eigen::VectorXd> Ws = WsDiag.asDiagonal();
		const Eigen::VectorXd sX = V*Ws.inverse()*U.adjoint()*b;
		if (sX.maxCoeff() <= 0)
			return sX;
	}

	// Extract submatrices in range and null space
	const Eigen::Block<const Eigen::MatrixXd> Us = U.topLeftCorner(N,K);
	//const Eigen::Block<const Eigen::MatrixXd> Ws = W.topLeftCorner(N,N);
	const Eigen::ArrayXd WdiagHead = Wdiag.head(K);
	const Eigen::VectorXd WsDiag(WdiagHead);
	const Eigen::DiagonalWrapper<const Eigen::VectorXd> Ws = WsDiag.asDiagonal();
	const Eigen::MatrixXd VT = V.transpose();
	const Eigen::Block<const Eigen::MatrixXd> Vs = V.topLeftCorner(N,K);
	const Eigen::Block<const Eigen::MatrixXd> VsT = VT.topLeftCorner(K,N);
	const Eigen::Block<const Eigen::MatrixXd> VsP = V.bottomRightCorner(N,N-K);
	const Eigen::Block<const Eigen::MatrixXd> VsPT = VT.bottomRightCorner(N-K,N);

	// Constant values
	const Eigen::MatrixXd Gs = Us*Ws;
	const Eigen::MatrixXd GsT = Ws*Us.transpose();
	const Eigen::VectorXd zeroVecN = Eigen::VectorXd::Zero(N);

	// Initialize
	static const double ALPHA = 0.01;
	static const double ALPHA_THRESHOLD = 1e-5;
	Eigen::VectorXd phi = Eigen::VectorXd::Zero(K);
	Eigen::VectorXd phiP = Eigen::VectorXd::Zero(N-K);
	Eigen::VectorXd sB = -b;
	Eigen::VectorXd sX = Eigen::VectorXd::Zero(N);
	Eigen::VectorXd xViolation = sX.cwiseMax(zeroVecN);
	for (Eigen::MatrixXd::Index i = 0; i < constraintDim; i++) {
		xViolation[i] = 0;
	}
    double fObj = 0.5*(sB.dot(sB)+xViolation.dot(xViolation));
    double a = ALPHA;
	unsigned int k;
	if (doLog)
		log->beginSection("Iterative Algorithm",RWPE_LOCATION);
	for (k = 0; k < iterations && fObj > eps; k++) {
    	if (fObj <= ALPHA_THRESHOLD && a > 0) {
    		a = 0;
    		if (doLog)
    			log->log("Alpha set to zero",RWPE_LOCATION) << "At iteration " << k;
    	}
    	xViolation = sX.cwiseMax(zeroVecN);
		for (Eigen::MatrixXd::Index i = 0; i < constraintDim; i++) {
			xViolation[i] = 0;
		}

    	const Eigen::VectorXd dsX = -xViolation+a*(-sX).cwiseMax(zeroVecN);
		const Eigen::VectorXd dsB = -sB;

		Eigen::VectorXd GammaDiagSq = Eigen::VectorXd::Constant(N,gammaMax*gammaMax);
		bool simple = true;
		for (Eigen::MatrixXd::Index i = constraintDim; i < N; i++) {
			if (sB[i] > 0) {
				GammaDiagSq[i] = GammaDiagSq[i]*std::min(1.,dsB.norm()/sB[i]);
				simple = false;
			}
		}
		const Eigen::DiagonalWrapper<const Eigen::VectorXd> GammaSq = GammaDiagSq.asDiagonal();
		Eigen::VectorXd dPhi;
		if (simple) {
			const Eigen::VectorXd BinvDiag( (WdiagHead.array().square()*(gammaMax*gammaMax) + 1).inverse() );
			const Eigen::DiagonalWrapper<const Eigen::VectorXd> Binv = BinvDiag.asDiagonal();
			dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
		} else {
			const Eigen::MatrixXd B = Eigen::MatrixXd::Identity(K,K)+GsT*GammaSq*Gs;
			const Eigen::MatrixXd Binv = B.inverse();
			dPhi = Binv*(GsT*GammaSq*dsB+VsT*dsX);
		}

		const Eigen::VectorXd dPhiP = VsPT*dsX;
		if (doLog) {
			std::ostream& lstr = log->log("Info",RWPE_LOCATION);
			lstr << "dsX: " << dsX << "\n";
			lstr << "dsB: " << dsB << "\n";
			lstr << "dPhi: " << dPhi << "\n";
			lstr << "dPhiP: " << dPhiP << "\n";
		}
		phi += dPhi;
		phiP += dPhiP;
		sB += Gs*dPhi;
		sX += Vs*dPhi+VsP*dPhiP;
    	fObj = dPhi.norm()+dPhiP.norm();
		/*if (doLog) {
			std::vector<std::string> labels;
			std::vector<double> values;
			labels.push_back("Collision Solver Iteration");
			labels.push_back("Collision Solver Precision");
			values.push_back(k);
			values.push_back(fObj);
			log->addValues("Iteration", values, labels, RWPE_LOCATION);
		}*/
    	if (doLog) {
    		log->log("sX",RWPE_LOCATION) << sX;
    	}
	}
	if (doLog)
		log->endSection(__LINE__);
	if (doLog) {
		std::vector<std::string> labels;
		std::vector<double> values;
		labels.push_back("Collision Solver Iterations");
		labels.push_back("Collision Solver Max Iterations");
		labels.push_back("Collision Solver Precision");
		labels.push_back("Collision Solver Precision Target");
		values.push_back(k);
		values.push_back(iterations);
		values.push_back(fObj);
		values.push_back(eps);
		log->addValues("Iterative SVD", values, labels, RWPE_LOCATION);
	}
	return sX;
}

void RWPECollisionSolverSimultaneous::applySolution(
	const Eigen::VectorXd& solution,
	const std::list<const RWPEBodyDynamic*>& component,
	const std::vector<const RWPEContact*>& contacts,
	const std::vector<const RWPEConstraint*>& constraints,
	const RWPEMaterialMap* map,
	RWPEIslandState& islandState,
	const State& rwstate)
{
	RW_ASSERT(contacts.size() > 0 || constraints.size() > 0);
	Eigen::MatrixXd::Index dimVel = 0;
	typedef std::pair<Vector3D<>, Wrench6D<> > Impulse;
	std::vector<std::list<Impulse> > impulses(component.size());
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity();
		const RWPEBody* const cParent = constraint->getParent();
		const RWPEBody* const cChild = constraint->getChild();
		const RWPEBodyDynamic* const rcParent = dynamic_cast<const RWPEBodyDynamic*>(cParent);
		const RWPEBodyDynamic* const rcChild = dynamic_cast<const RWPEBodyDynamic*>(cChild);

		if (dim > 0) {
			const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
			const Rotation3D<> linRot = constraint->getLinearRotationParentW(islandState);
			const Rotation3D<> angRot = constraint->getAngularRotationParentW(islandState);

			Vector3D<> impF = Vector3D<>::zero();
			Vector3D<> impN = Vector3D<>::zero();
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i] == RWPEConstraint::Velocity) {
					impF += solution[dimVel]*linRot.getCol(i);
					dimVel++;
				}
			}
			for (std::size_t i = 0; i < 3; i++) {
				if (modes[i+3] == RWPEConstraint::Velocity) {
					impN += solution[dimVel]*angRot.getCol(i);
					dimVel++;
				}
			}

			const Vector3D<> rij = constraint->getPositionParentW(islandState);
			const Vector3D<> rji = constraint->getPositionChildW(islandState);
			std::size_t i = 0;
			BOOST_FOREACH(const RWPEBodyDynamic* const body, component) {
				if (body == rcParent)
					impulses[i].push_back(std::make_pair(rij,Wrench6D<>(impF,impN)));
				else
					impulses[i].push_back(std::make_pair(rij,Wrench6D<>(-impF,-impN)));
				if (body == rcChild)
					impulses[i].push_back(std::make_pair(rji,Wrench6D<>(-impF,-impN)));
				else
					impulses[i].push_back(std::make_pair(rji,Wrench6D<>(impF,impN)));
				i++;
			}
		}
	}
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		const RWPEBody* const cParent = contact->getParent();
		const RWPEBody* const cChild = contact->getChild();
		const RWPEBodyDynamic* const rcParent = dynamic_cast<const RWPEBodyDynamic*>(cParent);
		const RWPEBodyDynamic* const rcChild = dynamic_cast<const RWPEBodyDynamic*>(cChild);

		const VelocityScrew6D<>& velI = contact->getVelocityParentW(islandState,rwstate);
		const VelocityScrew6D<>& velJ = contact->getVelocityChildW(islandState,rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> nij = contact->getNormalW(islandState);
		const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
		const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
		const bool leaving = dot(-linRelVel,nij) >= 0;

		RWPERestitutionModel::Values restitution;
		if (!leaving)
			restitution = map->getRestitutionModel(*contact).getRestitution(*contact,islandState,rwstate);

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

		const Vector3D<> rij = contact->getPositionParentW(islandState);
		const Vector3D<> rji = contact->getPositionChildW(islandState);
		std::size_t i = 0;
		BOOST_FOREACH(const RWPEBodyDynamic* const body, component) {
			if (body == rcParent)
				impulses[i].push_back(std::make_pair(rij,Wrench6D<>(impF,impN)));
			else if (body == rcChild)
				impulses[i].push_back(std::make_pair(rji,Wrench6D<>(-impF,-impN)));
			i++;
		}
	}
	std::size_t i = 0;
	BOOST_FOREACH(const RWPEBodyDynamic* const body, component) {
		BOOST_FOREACH(const Impulse& impulse, impulses[i]) {
			applyImpulse(impulse.second, impulse.first, *body, islandState);
		}
		i++;
	}
}


Eigen::MatrixXd RWPECollisionSolverSimultaneous::getBlock(
	const RWPEConstraint* constraintA,
	const RWPEConstraint* constraintB,
	const RWPEIslandState& islandState,
	const State& rwstate)
{
	const RWPEBody* const parentA = constraintA->getParent();
	const RWPEBody* const childA = constraintA->getChild();
	const RWPEBody* const parentB = constraintB->getParent();
	const RWPEBody* const childB = constraintB->getChild();

	const Eigen::VectorXd::Index dimA = constraintA->getDimVelocity();
	const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity();
	if (parentA != parentB && parentA != childB &&
			childA != parentB && childA != childB)
	{
		return Eigen::MatrixXd::Zero(dimA,dimB);
	}

	const RWPEBodyDynamic* const rParentA = dynamic_cast<const RWPEBodyDynamic*>(parentA);
	const RWPEBodyDynamic* const rChildA = dynamic_cast<const RWPEBodyDynamic*>(childA);
	const RWPEBodyDynamic* const rParentB = dynamic_cast<const RWPEBodyDynamic*>(parentB);
	const RWPEBodyDynamic* const rChildB = dynamic_cast<const RWPEBodyDynamic*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 6x6 matrix in world coordinates
	const Vector3D<> rij = constraintA->getPositionParentW(islandState);
	const Vector3D<> rji = constraintA->getPositionChildW(islandState);
	const Transform3D<>& wTbI = parentA->getWorldTcom(islandState);
	const Transform3D<>& wTbJ = childA->getWorldTcom(islandState);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();
	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = constraintB->getPositionParentW(islandState);
	else
		rik = constraintB->getPositionChildW(islandState);
	if (childA == parentB)
		rjk = constraintB->getPositionParentW(islandState);
	else
		rjk = constraintB->getPositionChildW(islandState);

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
	const std::vector<RWPEConstraint::Mode> modesA = constraintA->getConstraintModes();
	const std::vector<RWPEConstraint::Mode> modesB = constraintB->getConstraintModes();
	const Rotation3D<> linRotA = constraintA->getLinearRotationParentW(islandState);
	const Rotation3D<> angRotA = constraintA->getAngularRotationParentW(islandState);
	const Rotation3D<> linRotB = constraintB->getLinearRotationParentW(islandState);
	const Rotation3D<> angRotB = constraintB->getAngularRotationParentW(islandState);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	for (std::size_t i = 0; i < 3; i++) {
		if (modesA[i] == RWPEConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == RWPEConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*BFlin*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == RWPEConstraint::Velocity) {
					block(row,col) = linRotA.getCol(i).e().transpose()*bNlin*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}
	for (std::size_t i = 3; i < 6; i++) {
		if (modesA[i] == RWPEConstraint::Velocity) {
			col = 0;
			for (std::size_t j = 0; j < 3; j++) {
				if (modesB[j] == RWPEConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*BFang*linRotB.getCol(j).e();
					col++;
				}
			}
			for (std::size_t j = 3; j < 6; j++) {
				if (modesB[j] == RWPEConstraint::Velocity) {
					block(row,col) = angRotA.getCol(i-3).e().transpose()*bNang*angRotB.getCol(j-3).e();
					col++;
				}
			}
			row++;
		}
	}

	return block;
}

Eigen::MatrixXd RWPECollisionSolverSimultaneous::getBlock(
	const RWPEConstraint* constraintA,
	const RWPEContact* contactB,
	const RWPERestitutionModel& restitutionModel,
	const RWPEIslandState& islandState,
	const State& rwstate)
{
	const RWPEBody* const parentA = constraintA->getParent();
	const RWPEBody* const childA = constraintA->getChild();
	const RWPEBody* const parentB = contactB->getParent();
	const RWPEBody* const childB = contactB->getChild();

	const VelocityScrew6D<>& velIB = contactB->getVelocityParentW(islandState,rwstate);
	const VelocityScrew6D<>& velJB = contactB->getVelocityChildW(islandState,rwstate);
	const Vector3D<> linRelVelB = velIB.linear()-velJB.linear();
	const Vector3D<> nijB = contactB->getNormalW(islandState);
	const bool leavingB = dot(-linRelVelB,nijB) >= 0;

	RWPERestitutionModel::Values restitutionB;
	if (!leavingB)
		restitutionB = restitutionModel.getRestitution(*contactB,islandState,rwstate);

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

	const RWPEBodyDynamic* const rParentA = dynamic_cast<const RWPEBodyDynamic*>(parentA);
	const RWPEBodyDynamic* const rChildA = dynamic_cast<const RWPEBodyDynamic*>(childA);
	const RWPEBodyDynamic* const rParentB = dynamic_cast<const RWPEBodyDynamic*>(parentB);
	const RWPEBodyDynamic* const rChildB = dynamic_cast<const RWPEBodyDynamic*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 6x4 matrix in world coordinates
	const Vector3D<> rij = constraintA->getPositionParentW(islandState);
	const Vector3D<> rji = constraintA->getPositionChildW(islandState);
	const Transform3D<>& wTbI = parentA->getWorldTcom(islandState);
	const Transform3D<>& wTbJ = childA->getWorldTcom(islandState);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	const Vector3D<> zeroDirB = normalize(cross(nijB,linRelVelB));
	const Vector3D<> tangentDirB = normalize(cross(zeroDirB,nijB));

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = contactB->getPositionParentW(islandState);
	else
		rik = contactB->getPositionChildW(islandState);
	if (childA == parentB)
		rjk = contactB->getPositionParentW(islandState);
	else
		rjk = contactB->getPositionChildW(islandState);

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
	const std::vector<RWPEConstraint::Mode> modesA = constraintA->getConstraintModes();
	const Rotation3D<> linRotA = constraintA->getLinearRotationParentW(islandState);
	const Rotation3D<> angRotA = constraintA->getAngularRotationParentW(islandState);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;

	for (std::size_t i = 0; i < 3; i++) {
		if (modesA[i] == RWPEConstraint::Velocity) {
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
		if (modesA[i] == RWPEConstraint::Velocity) {
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

Eigen::MatrixXd RWPECollisionSolverSimultaneous::getBlock(
	const RWPEContact* contactA,
	const RWPEConstraint* constraintB,
	const RWPERestitutionModel& restitutionModel,
	const RWPEIslandState& islandState,
	const State& rwstate)
{
	const RWPEBody* const parentA = contactA->getParent();
	const RWPEBody* const childA = contactA->getChild();
	const RWPEBody* const parentB = constraintB->getParent();
	const RWPEBody* const childB = constraintB->getChild();

	const VelocityScrew6D<>& velI = contactA->getVelocityParentW(islandState,rwstate);
	const VelocityScrew6D<>& velJ = contactA->getVelocityChildW(islandState,rwstate);
	const Vector3D<> linRelVel = velI.linear()-velJ.linear();
	const Vector3D<> nij = contactA->getNormalW(islandState);
	const bool leavingA = dot(-linRelVel,nij) >= 0;

	RWPERestitutionModel::Values restitutionA;
	if (!leavingA)
		restitutionA = restitutionModel.getRestitution(*contactA,islandState,rwstate);

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

	const RWPEBodyDynamic* const rParentA = dynamic_cast<const RWPEBodyDynamic*>(parentA);
	const RWPEBodyDynamic* const rChildA = dynamic_cast<const RWPEBodyDynamic*>(childA);
	const RWPEBodyDynamic* const rParentB = dynamic_cast<const RWPEBodyDynamic*>(parentB);
	const RWPEBodyDynamic* const rChildB = dynamic_cast<const RWPEBodyDynamic*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 4x6 matrix in world coordinates
	const Vector3D<> rij = contactA->getPositionParentW(islandState);
	const Vector3D<> rji = contactA->getPositionChildW(islandState);
	const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
	const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
	const Transform3D<>& wTbI = parentA->getWorldTcom(islandState);
	const Transform3D<>& wTbJ = childA->getWorldTcom(islandState);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = constraintB->getPositionParentW(islandState);
	else
		rik = constraintB->getPositionChildW(islandState);
	if (childA == parentB)
		rjk = constraintB->getPositionParentW(islandState);
	else
		rjk = constraintB->getPositionChildW(islandState);

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
	const std::vector<RWPEConstraint::Mode> modesB = constraintB->getConstraintModes();
	const Rotation3D<> linRotB = constraintB->getLinearRotationParentW(islandState);
	const Rotation3D<> angRotB = constraintB->getAngularRotationParentW(islandState);

	Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimA,dimB);
	Eigen::MatrixXd::Index row = 0;
	Eigen::MatrixXd::Index col = 0;
	if (!leavingA && restitutionA.enableTangent) {
		col = 0;
		for (std::size_t j = 0; j < 3; j++) {
			if (modesB[j] == RWPEConstraint::Velocity) {
				block(row+0,col) = tangentDir.e().transpose()*BFlin*linRotB.getCol(j).e();
				block(row+1,col) = zeroDir.e().transpose()*BFlin*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == RWPEConstraint::Velocity) {
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
			if (modesB[j] == RWPEConstraint::Velocity) {
				block(row,col) = nij.e().transpose()*BFlin*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == RWPEConstraint::Velocity) {
				block(row,col) = nij.e().transpose()*bNlin*angRotB.getCol(j-3).e();
				col++;
			}
		}
		row++;
	}
	col = 0;
	if (!leavingA && restitutionA.enableAngular) {
		for (std::size_t j = 0; j < 3; j++) {
			if (modesB[j] == RWPEConstraint::Velocity) {
				block(row,col) = BFang*linRotB.getCol(j).e();
				col++;
			}
		}
		for (std::size_t j = 3; j < 6; j++) {
			if (modesB[j] == RWPEConstraint::Velocity) {
				block(row,col) = bNang*angRotB.getCol(j-3).e();
				col++;
			}
		}
		row++;
	}

	return block;
}

Eigen::MatrixXd RWPECollisionSolverSimultaneous::getBlock(
	const RWPEContact* contactA,
	const RWPEContact* contactB,
	const RWPERestitutionModel& restitutionModelA,
	const RWPERestitutionModel& restitutionModelB,
	const RWPEIslandState& islandState,
	const State& rwstate)
{
	const RWPEBody* const parentA = contactA->getParent();
	const RWPEBody* const childA = contactA->getChild();
	const RWPEBody* const parentB = contactB->getParent();
	const RWPEBody* const childB = contactB->getChild();

	const VelocityScrew6D<>& velI = contactA->getVelocityParentW(islandState,rwstate);
	const VelocityScrew6D<>& velJ = contactA->getVelocityChildW(islandState,rwstate);
	const Vector3D<> linRelVel = velI.linear()-velJ.linear();
	const Vector3D<> nij = contactA->getNormalW(islandState);
	const bool leavingA = dot(-linRelVel,nij) >= 0;

	const VelocityScrew6D<>& velIB = contactB->getVelocityParentW(islandState,rwstate);
	const VelocityScrew6D<>& velJB = contactB->getVelocityChildW(islandState,rwstate);
	const Vector3D<> linRelVelB = velIB.linear()-velJB.linear();
	const Vector3D<> nijB = contactB->getNormalW(islandState);
	const bool leavingB = dot(-linRelVelB,nijB) >= 0;

	RWPERestitutionModel::Values restitutionA;
	RWPERestitutionModel::Values restitutionB;
	if (!leavingA)
		restitutionA = restitutionModelA.getRestitution(*contactA,islandState,rwstate);
	if (!leavingB)
		restitutionB = restitutionModelB.getRestitution(*contactB,islandState,rwstate);

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

	const RWPEBodyDynamic* const rParentA = dynamic_cast<const RWPEBodyDynamic*>(parentA);
	const RWPEBodyDynamic* const rChildA = dynamic_cast<const RWPEBodyDynamic*>(childA);
	const RWPEBodyDynamic* const rParentB = dynamic_cast<const RWPEBodyDynamic*>(parentB);
	const RWPEBodyDynamic* const rChildB = dynamic_cast<const RWPEBodyDynamic*>(childB);
	int sign;
	if (rParentA == rParentB || rChildA == rChildB)
		sign = 1;
	else
		sign = -1;

	// Construct the full 4x4 matrix in world coordinates
	const Vector3D<> rij = contactA->getPositionParentW(islandState);
	const Vector3D<> rji = contactA->getPositionChildW(islandState);
	const Vector3D<> zeroDir = normalize(cross(nij,linRelVel));
	const Vector3D<> tangentDir = normalize(cross(zeroDir,nij));
	const Transform3D<>& wTbI = parentA->getWorldTcom(islandState);
	const Transform3D<>& wTbJ = childA->getWorldTcom(islandState);
	const Vector3D<>& RI = wTbI.P();
	const Vector3D<>& RJ = wTbJ.P();

	const Vector3D<> zeroDirB = normalize(cross(nijB,linRelVelB));
	const Vector3D<> tangentDirB = normalize(cross(zeroDirB,nijB));

	Vector3D<> rik, rjk;
	if (parentA == parentB)
		rik = contactB->getPositionParentW(islandState);
	else
		rik = contactB->getPositionChildW(islandState);
	if (childA == parentB)
		rjk = contactB->getPositionParentW(islandState);
	else
		rjk = contactB->getPositionChildW(islandState);

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
