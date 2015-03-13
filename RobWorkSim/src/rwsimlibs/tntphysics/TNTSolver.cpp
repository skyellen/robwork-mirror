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

#include "TNTSolver.hpp"
#include "TNTSolverSVD.hpp"
#include "TNTSettings.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraint.hpp"
#include "TNTRigidBody.hpp"
#include "TNTIslandState.hpp"
#include "TNTContact.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_ENABLE_DEBUG "TNTSolverDebug"

TNTSolver::TNTSolver(const TNTBodyConstraintManager* manager, const Vector3D<double> &gravity):
	_manager(manager),
	_gravity(gravity)
{
}

Eigen::VectorXd TNTSolver::solve(double h, const State &rwstate, const TNTIslandState &tntstate, const PropertyMap& pmap) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	// First find the properties to use
	int DEBUG = pmap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
#if !TNT_DEBUG_ENABLE_SOLVER
	DEBUG = 0;
#endif
	if (DEBUG != 0 && DEBUG != 1) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (DEBUG < 0)
			DEBUG = tmpMap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
		RW_ASSERT(DEBUG == 0 || DEBUG == 1);
	}

	Eigen::MatrixXd lhs;
	Eigen::VectorXd rhs;
	getMatrices(lhs, rhs, h, rwstate, tntstate, DEBUG == 1);
	const Eigen::VectorXd sol = solve(lhs, rhs, pmap);
	RW_ASSERT(lhs.rows() == sol.size());
	return sol;
}

void TNTSolver::getMatrices(Eigen::MatrixXd& lhs, Eigen::VectorXd& rhs, double h, const State &rwstate, const TNTIslandState &tntstate, bool debug) const {
	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimCon = 0;
	Eigen::MatrixXd::Index dimVar = 0;
	const TNTBodyConstraintManager::ConstraintList constraints = _manager->getConstraints(tntstate);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		if (dynParent || dynChild) {
			dimCon += constraint->getDimVelocity()+constraint->getDimWrench();
			//dimVar += constraint->getDimVelocity()+constraint->getDimWrench() > 0 ? 6 : 0;
			dimVar += constraint->getDimVelocity()+constraint->getDimWrench();
		}
	}
	lhs = Eigen::MatrixXd::Zero(dimCon,dimVar);
	rhs = Eigen::VectorXd::Zero(dimCon);

	if (debug) {
		TNT_DEBUG_SOLVER("Constructing constraint equation system of dimensions " << dimCon << " x " << dimVar << ".");
		{
			std::stringstream sstr;
			sstr << "Contact types: ";
			BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
				const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
				if (contact) {
					if (contact->isLeaving())
						sstr << "Leaving ";
					else
						sstr << "(" << contact->toString(contact->getTypeLinear()) << "," << contact->toString(contact->getTypeAngular()) << ") ";
				}
			}
			TNT_DEBUG_SOLVER(sstr.str());
		}
	}

	// Construct the matrices
	dimCon = 0;
	dimVar = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity()+constraint->getDimWrench();
		if (dim > 0 && (dynParent || dynChild)) {
			Eigen::VectorXd aij = constraint->getRHS(h, _gravity, constraints, rwstate, tntstate);
			rhs.block(dimCon,0,dim,1) = aij;
			Eigen::MatrixXd::Index dimVarB = 0;
			BOOST_FOREACH(const TNTConstraint* constraintB, constraints) {
				//const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity()+constraintB->getDimWrench() > 0 ? 6 : 0;
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity()+constraintB->getDimWrench();
				if (dimB > 0) {
					const TNTBody* const cP = constraint->getParent();
					const TNTBody* const cPB = constraintB->getParent();
					const TNTBody* const cC = constraint->getChild();
					const TNTBody* const cCB = constraintB->getChild();
					if (cP == cPB || cP == cCB || cC == cPB || cC == cCB) {
						Eigen::MatrixXd B = constraint->getLHS(constraintB, h, rwstate, tntstate);
						lhs.block(dimCon,dimVarB,dim,dimB) = B;
					}
					dimVarB += dimB;
				}
			}
			dimCon += dim;
		}
	}
}

void TNTSolver::saveSolution(const Eigen::VectorXd& solution, TNTIslandState &state) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (saveSolution): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");
	std::size_t cI = 0;
	const TNTBodyConstraintManager::ConstraintList constraints = _manager->getConstraints(state);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		// Restore full wrench vectors
		const std::vector<TNTConstraint::Mode> constraintModes = constraint->getConstraintModes();
		//const Eigen::MatrixXd::Index dimVar = constraint->getDimVelocity() + constraint->getDimWrench() > 0 ? 6 : 0;
		const Eigen::MatrixXd::Index dimVar = constraint->getDimVelocity() + constraint->getDimWrench();
		if (dimVar > 0) {
			Vector3D<> velLin, velAng;
			for (std::size_t i = 0; i < 3; i++) {
				const TNTConstraint::Mode &mode = constraintModes[i];
				if (mode == TNTConstraint::Velocity || mode == TNTConstraint::Wrench) {
					RW_ASSERT((int)cI < solution.rows());
					velLin[i] = solution[cI];
					cI ++;
				}
			}
			for (std::size_t i = 3; i < 6; i++) {
				const TNTConstraint::Mode &mode = constraintModes[i];
				if (mode == TNTConstraint::Velocity || mode == TNTConstraint::Wrench) {
					RW_ASSERT((int)cI < solution.rows());
					velAng[i-3] = solution[cI];
					cI ++;
				}
			}
			// Rotate back to world frame
			const Rotation3D<> Rlin = constraint->getLinearRotationParentW(state);
			const Rotation3D<> Rang = constraint->getAngularRotationParentW(state);
			const Wrench6D<> wrench(Rlin*velLin,Rang*velAng);
			state.setWrenchConstraint(constraint,wrench);
		}
	}
	TNT_DEBUG_SOLVER("Forces & Torques applied.");
}

const TNTBodyConstraintManager* TNTSolver::getManager() const {
	return _manager;
}

const Vector3D<>& TNTSolver::getGravity() const {
	return _gravity;
}

void TNTSolver::addDefaultProperties(PropertyMap& map) const {
	map.add<int>(PROPERTY_ENABLE_DEBUG,"Enable or disable debugging (really slow).",1);
}

TNTSolver::Factory::Factory():
	ExtensionPoint<TNTSolver>("rwsimlibs.tntphysics.TNTSolver", "TNTSolver extension point.")
{
}

std::vector<std::string> TNTSolver::Factory::getSolvers() {
	std::vector<std::string> solvers;
	solvers.push_back("SVD");
	TNTSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		solvers.push_back( ext.getProperties().get("solverID",ext.name) );
	}
	return solvers;
}

bool TNTSolver::Factory::hasSolver(const std::string& solverType) {
	if (solverType == "SVD")
		return true;
	TNTSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("solverID",ext.name) == solverType)
            return true;
	}
	return false;
}

const TNTSolver* TNTSolver::Factory::makeSolver(const std::string& solverType, const TNTBodyConstraintManager* manager, const Vector3D<double> &gravity) {
	if (solverType == "SVD")
		return new TNTSolverSVD(manager,gravity);
	TNTSolver::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == solverType){
			const rw::common::Ptr<const TNTSolver> base = ext->getObject().cast<const TNTSolver>();
			return base->createSolver(manager,gravity);
		}
	}
	return NULL;
}
