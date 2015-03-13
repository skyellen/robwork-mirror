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

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_SVD_PRECISION "TNTSolverSVDPrecision"
#define PROPERTY_ENABLE_DEBUG "TNTSolverDebug"

TNTSolverSVD::TNTSolverSVD():
	TNTSolver(NULL,Vector3D<>::zero())
{
}

TNTSolverSVD::TNTSolverSVD(const TNTBodyConstraintManager* manager, const Vector3D<double> &gravity):
	TNTSolver(manager,gravity)
{
}

TNTSolverSVD::~TNTSolverSVD() {
}

const TNTSolver* TNTSolverSVD::createSolver(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) const {
	return new TNTSolverSVD(manager,gravity);
}

Eigen::VectorXd TNTSolverSVD::solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const PropertyMap& pmap) const {
	if (_manager == NULL)
		RW_THROW("TNTSolverSVD (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	// First find the properties to use
	double PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);
	int DEBUG = pmap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
#if !TNT_DEBUG_ENABLE_SOLVER
	DEBUG = 0;
#endif
	if (PRECISION < 0 || (DEBUG != 0 && DEBUG != 1)) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (PRECISION < 0)
			PRECISION = tmpMap.get<double>(PROPERTY_SVD_PRECISION,-1.);
		RW_ASSERT(PRECISION > 0);
		if (DEBUG < 0)
			DEBUG = tmpMap.get<int>(PROPERTY_ENABLE_DEBUG,-1.);
		RW_ASSERT(DEBUG == 0 || DEBUG == 1);
	}

	// Solution
	Eigen::VectorXd solution;
	RW_ASSERT(A.rows() == b.rows());
	if (A.rows() > 0) {
		solution = LinearAlgebra::pseudoInverse(A,PRECISION)*b;
		if (DEBUG == 1) {
			const Eigen::VectorXd residual = A*solution-b;
			TNT_DEBUG_SOLVER("Forces & Torques found with norm-2 residual " << residual.norm() << ".");
			if (residual.norm() > 1e-10)
				TNT_DEBUG_SOLVER("Forces & Torques found with residuals " << residual.transpose() << ".");
			/*std::pair<Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<std::complex<double>, Eigen::Dynamic, -1> > dec = LinearAlgebra::eigenDecomposition(lhs);
			Eigen::VectorXd eigenValues(dec.second.rows());
			for (Eigen::VectorXd::Index i = 0; i < dec.second.rows(); i++) {
				const std::complex<double> &complex = dec.second(i,0);
				eigenValues[i] = complex.real();
			}
			TNT_DEBUG_SOLVER("Eigenvalues: " << eigenValues.transpose() << ".");*/
			TNT_DEBUG_SOLVER("LHS: " << std::endl << A);
			TNT_DEBUG_SOLVER("RHS: " << b.transpose() << ".");
			TNT_DEBUG_SOLVER("Solution: " << solution.transpose() << ".");
			TNT_DEBUG_SOLVER("Residual: " << residual.transpose() << ".");
		}
	} else
		solution = Eigen::VectorXd::Zero(0);
	return solution;
}

void TNTSolverSVD::addDefaultProperties(PropertyMap& map) const {
	TNTSolver::addDefaultProperties(map);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
	map.add<int>(PROPERTY_ENABLE_DEBUG,"Enable or disable debugging (really slow).",1);
}
