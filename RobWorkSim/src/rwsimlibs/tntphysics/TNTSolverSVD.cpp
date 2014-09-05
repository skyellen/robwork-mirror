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

#include "TNTSolverSVD.hpp"

#include "TNTSettings.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraint.hpp"
#include "TNTRigidBody.hpp"
#include "TNTIslandState.hpp"

#include <boost/foreach.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#include <rwsim/dynamics/Body.hpp>

TNTSolverSVD::TNTSolverSVD():
	_manager(NULL)
{
}

TNTSolverSVD::TNTSolverSVD(const TNTBodyConstraintManager* manager, const Vector3D<double> &gravity):
	_manager(manager),
	_gravity(gravity)
{
}

TNTSolverSVD::~TNTSolverSVD() {
}

const TNTSolver* TNTSolverSVD::createSolver(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) const {
	return new TNTSolverSVD(manager,gravity);
}

Eigen::VectorXd TNTSolverSVD::solve(double h, const State &rwstate, const TNTIslandState &tntstate) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");
	// Allocate new matrix and vector structures
	Eigen::MatrixXd::Index dimVel = 0;
	const TNTBodyConstraintManager::ConstraintList constraints = _manager->getConstraints(tntstate);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		if (dynParent || dynChild) {
			dimVel += constraint->getDimVelocity()+constraint->getDimWrench();
		}
	}
	Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(dimVel,dimVel);
	Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dimVel);
	TNT_DEBUG_SOLVER("Constructing constraint equation system of dimensions " << dimVel << " x " << dimVel << ".");

	// Construct the matrices
	dimVel = 0;
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		const bool dynParent = dynamic_cast<const TNTRigidBody*>(constraint->getParent());
		const bool dynChild = dynamic_cast<const TNTRigidBody*>(constraint->getChild());
		const Eigen::VectorXd::Index dim = constraint->getDimVelocity()+constraint->getDimWrench();
		if (dim > 0 && (dynParent || dynChild)) {
			Eigen::VectorXd aij = constraint->getRHS(h, _gravity, constraints, rwstate, tntstate);
			rhs.block(dimVel,0,dim,1) = aij;
			Eigen::MatrixXd::Index dimVelB = 0;
			BOOST_FOREACH(const TNTConstraint* constraintB, constraints) {
				const Eigen::VectorXd::Index dimB = constraintB->getDimVelocity()+constraintB->getDimWrench();
				if (dimB > 0) {
					const TNTBody* const cP = constraint->getParent();
					const TNTBody* const cPB = constraintB->getParent();
					const TNTBody* const cC = constraint->getChild();
					const TNTBody* const cCB = constraintB->getChild();
					if (cP == cPB || cP == cCB || cC == cPB || cC == cCB) {
						Eigen::MatrixXd B = constraint->getLHS(constraintB, h, rwstate, tntstate);
						lhs.block(dimVel,dimVelB,dim,dimB) = B;
					}
					dimVelB += dimB;
				}
			}
			dimVel += dim;
		}
	}

	// Solution
	Eigen::VectorXd solution;
	RW_ASSERT(lhs.rows() == rhs.rows());
	if (lhs.rows() > 0) {
		solution = LinearAlgebra::pseudoInverse(lhs,TNT_SVD_PRECISSION)*rhs;
#ifdef TNT_DEBUG_ENABLE_SOLVER
		const Eigen::VectorXd residual = lhs*solution-rhs;
		TNT_DEBUG_SOLVER("Forces & Torques found with norm-2 residual " << residual.norm() << ".");
		if (residual.norm() > 1e-10)
			TNT_DEBUG_SOLVER("Forces & Torques found with residuals " << residual.transpose() << ".");
		std::pair<Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<std::complex<double>, Eigen::Dynamic, -1> > dec = LinearAlgebra::eigenDecomposition(lhs);
		Eigen::VectorXd eigenValues(dec.second.rows());
		for (Eigen::VectorXd::Index i = 0; i < dec.second.rows(); i++) {
			const std::complex<double> &complex = dec.second(i,0);
			eigenValues[i] = complex.real();
		}
		TNT_DEBUG_SOLVER("Eigenvalues: " << eigenValues.transpose() << ".");
		TNT_DEBUG_SOLVER("RHS: " << rhs.transpose() << ".");
#endif
	} else
		solution = Eigen::VectorXd::Zero(0);
	return solution;
}

void TNTSolverSVD::saveSolution(const Eigen::VectorXd& solution, TNTIslandState &state) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (saveSolution): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");
	std::size_t cI = 0;
	const TNTBodyConstraintManager::ConstraintList constraints = _manager->getConstraints(state);
	BOOST_FOREACH(const TNTConstraint* constraint, constraints) {
		// Restore full wrench vectors
		const std::vector<TNTConstraint::Mode> constraintModes = constraint->getConstraintModes();
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
	TNT_DEBUG_SOLVER("Forces & Torques applied.");
}

const TNTBodyConstraintManager* TNTSolverSVD::getManager() const {
	return _manager;
}

const Vector3D<>& TNTSolverSVD::getGravity() const {
	return _gravity;
}
