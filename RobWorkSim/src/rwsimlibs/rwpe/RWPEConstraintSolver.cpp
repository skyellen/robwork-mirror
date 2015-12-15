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

#include <boost/foreach.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEConstraintSolver.hpp"

#include "RWPEConstraint.hpp"
#include "RWPEConstraintSolverDirect.hpp"
#include "RWPEConstraintSolverIterative.hpp"
#include "RWPEContact.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

#define PROPERTY_ENABLE_DEBUG "RWPEConstraintSolverDebug"

RWPEConstraintSolver::RWPEConstraintSolver(const RWPEBodyConstraintGraph* manager, const Vector3D<double> &gravity):
	_manager(manager),
	_gravity(gravity)
{
}

Eigen::VectorXd RWPEConstraintSolver::solve(double h, bool discontinuity, const State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH, const PropertyMap& pmap, RWPELogUtil* log) const {
	if (_manager == NULL)
		RW_THROW("RWPEConstraintSolverDirect (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	// First find the properties to use
	int DEBUG = pmap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
	if (DEBUG != 0 && DEBUG != 1) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (DEBUG < 0)
			DEBUG = tmpMap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
		RW_ASSERT(DEBUG == 0 || DEBUG == 1);
	}

	Eigen::MatrixXd lhs;
	Eigen::VectorXd rhs;
	Eigen::MatrixXd::Index constraintDim;
	getMatrices(lhs, rhs, constraintDim, h, discontinuity, rwstate, islandState0, islandStateH, DEBUG == 1, log);
	const Eigen::VectorXd sol = solve(lhs, rhs, constraintDim, pmap, log);
	RW_ASSERT(lhs.rows() == sol.size());
	return sol;
}

void RWPEConstraintSolver::getMatrices(Eigen::MatrixXd& lhs, Eigen::VectorXd& rhs, Eigen::MatrixXd::Index& constraintDim, double h, bool discontinuity, const State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH, bool debug, RWPELogUtil* log) const {
	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimCon = 0;
	Eigen::MatrixXd::Index dimVar = 0;
	constraintDim = 0;
	const RWPEBodyConstraintGraph::ConstraintList constraints0 = _manager->getConstraints(islandState0);
	const RWPEBodyConstraintGraph::ConstraintList constraints = _manager->getConstraints(islandStateH);
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		if (dynParent || dynChild) {
			dimCon += constraint->getDimVelocity()+constraint->getDimWrench();
			//dimVar += constraint->getDimVelocity()+constraint->getDimWrench() > 0 ? 6 : 0;
			dimVar += constraint->getDimVelocity()+constraint->getDimWrench();
			if (!dynamic_cast<const RWPEContact*>(constraint)) {
				constraintDim++;
			}
		}
	}
	lhs = Eigen::MatrixXd::Zero(dimCon,dimVar);
	rhs = Eigen::VectorXd::Zero(dimCon);

	if (debug) {
		log->log("Equation System Dimensions",RWPE_LOCATION) << "Constructing constraint equation system of dimensions " << dimCon << " x " << dimVar << ".";
		{
			std::ostream& lstr = log->log(RWPE_LOCATION);
			lstr << "Contact types: ";
			BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
				const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
				if (contact) {
					if (contact->isLeaving()) {
						lstr << "Leaving ";
					} else {
						lstr << "(" << contact->toString(contact->getTypeLinear()) << "," << contact->toString(contact->getTypeAngular()) << ") ";
					}
				}
			}
		}
	}

	// Construct the matrices
	dimCon = 0;
	dimVar = 0;
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity()+constraint->getDimWrench();
		if (dim > 0 && (dynParent || dynChild)) {
			Eigen::VectorXd aij = constraint->getRHS(h, _gravity, discontinuity, constraints0, constraints, rwstate, islandState0, islandStateH);
			RW_ASSERT(aij.rows() == dim);
			RW_ASSERT(dimCon+dim <= rhs.rows());
			rhs.block(dimCon,0,dim,1) = aij;
			Eigen::MatrixXd::Index dimVarB = 0;
			BOOST_FOREACH(const RWPEConstraint* constraintB, constraints) {
				const bool dynParentB = dynamic_cast<const RWPEBodyDynamic*>(constraintB->getParent());
				const bool dynChildB = dynamic_cast<const RWPEBodyDynamic*>(constraintB->getChild());
				//const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity()+constraintB->getDimWrench() > 0 ? 6 : 0;
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity()+constraintB->getDimWrench();
				if (dimB > 0 && (dynParentB || dynChildB)) {
					const RWPEBody* const cP = constraint->getParent();
					const RWPEBody* const cPB = constraintB->getParent();
					const RWPEBody* const cC = constraint->getChild();
					const RWPEBody* const cCB = constraintB->getChild();
					if (cP == cPB || cP == cCB || cC == cPB || cC == cCB) {
						Eigen::MatrixXd B = constraint->getLHS(constraintB, h, discontinuity, rwstate, islandStateH);
						RW_ASSERT(B.rows() == dim);
						RW_ASSERT(B.cols() == dimB);
						RW_ASSERT(dimVarB+dimB <= lhs.cols());
						lhs.block(dimCon,dimVarB,dim,dimB) = B;
					}
					dimVarB += dimB;
				}
			}
			dimCon += dim;
		}
	}
}

void RWPEConstraintSolver::saveSolution(const Eigen::VectorXd& solution, RWPEIslandState &state, RWPELogUtil* log) const {
	if (_manager == NULL)
		RW_THROW("RWPEConstraintSolverDirect (saveSolution): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");
	bool doLog = false;
	if (log != NULL)
		doLog = log->doLog();
	std::size_t cI = 0;
	const RWPEBodyConstraintGraph::ConstraintList constraints = _manager->getConstraints(state);
	BOOST_FOREACH(const RWPEConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const RWPEBodyDynamic*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const RWPEBodyDynamic*>(constraint->getChild());
		// Restore full wrench vectors
		const std::vector<RWPEConstraint::Mode> constraintModes = constraint->getConstraintModes();
		//const Eigen::MatrixXd::Index dimVar = constraint->getDimVelocity() + constraint->getDimWrench() > 0 ? 6 : 0;
		const Eigen::MatrixXd::Index dimVar = constraint->getDimVelocity() + constraint->getDimWrench();
		if (dimVar > 0 && (dynParent || dynChild)) {
			Vector3D<> velLin, velAng;
			for (std::size_t i = 0; i < 3; i++) {
				const RWPEConstraint::Mode &mode = constraintModes[i];
				if (mode == RWPEConstraint::Velocity || mode == RWPEConstraint::Wrench) {
					RW_ASSERT((int)cI < solution.rows());
					velLin[i] = solution[cI];
					cI ++;
				}
			}
			for (std::size_t i = 3; i < 6; i++) {
				const RWPEConstraint::Mode &mode = constraintModes[i];
				if (mode == RWPEConstraint::Velocity || mode == RWPEConstraint::Wrench) {
					RW_ASSERT((int)cI < solution.rows());
					velAng[i-3] = solution[cI];
					cI ++;
				}
			}
			// Rotate back to world frame
			const Rotation3D<> Rlin = constraint->getLinearRotationParentForceW(state);
			const Rotation3D<> Rang = constraint->getAngularRotationParentW(state);
			const Wrench6D<> wrench(Rlin*velLin,Rang*velAng);
			state.setWrenchConstraint(constraint,wrench);
		}
	}
	if (doLog)
		log->log("Forces & Torques Applied",RWPE_LOCATION);
}

const RWPEBodyConstraintGraph* RWPEConstraintSolver::getManager() const {
	return _manager;
}

const Vector3D<>& RWPEConstraintSolver::getGravity() const {
	return _gravity;
}

void RWPEConstraintSolver::addDefaultProperties(PropertyMap& map) const {
	map.add<int>(PROPERTY_ENABLE_DEBUG,"Enable or disable debugging (really slow).",1);
}

RWPEConstraintSolver::Factory::Factory():
	ExtensionPoint<RWPEConstraintSolver>("rwsimlibs.rwpe.RWPEConstraintSolver", "RWPEConstraintSolver extension point.")
{
}

std::vector<std::string> RWPEConstraintSolver::Factory::getSolvers() {
	std::vector<std::string> solvers;
	solvers.push_back("SVD");
	solvers.push_back("IterativeSVD");
	RWPEConstraintSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		solvers.push_back( ext.getProperties().get("solverID",ext.name) );
	}
	return solvers;
}

bool RWPEConstraintSolver::Factory::hasSolver(const std::string& solverType) {
	if (solverType == "SVD")
		return true;
	else if (solverType == "IterativeSVD")
		return true;
	RWPEConstraintSolver::Factory factory;
	std::vector<Extension::Descriptor> exts = factory.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
        if(ext.getProperties().get("solverID",ext.name) == solverType)
            return true;
	}
	return false;
}

const RWPEConstraintSolver* RWPEConstraintSolver::Factory::makeSolver(const std::string& solverType, const RWPEBodyConstraintGraph* manager, const Vector3D<double> &gravity) {
	if (solverType == "Direct")
		return new RWPEConstraintSolverDirect(manager,gravity);
	else if (solverType == "Iterative")
		return new RWPEConstraintSolverIterative(manager,gravity);
	RWPEConstraintSolver::Factory factory;
	std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getProperties().get("solverID",ext->getName() ) == solverType){
			const rw::common::Ptr<const RWPEConstraintSolver> base = ext->getObject().cast<const RWPEConstraintSolver>();
			return base->createSolver(manager,gravity);
		}
	}
	return NULL;
}
