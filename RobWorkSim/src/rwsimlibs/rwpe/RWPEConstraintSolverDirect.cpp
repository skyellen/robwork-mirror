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
#include "RWPEConstraintSolverDirect.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

#define PROPERTY_SVD_PRECISION "RWPEConstraintSolverDirectPrecision"

RWPEConstraintSolverDirect::RWPEConstraintSolverDirect():
	RWPEConstraintSolver(NULL,Vector3D<>::zero())
{
}

RWPEConstraintSolverDirect::RWPEConstraintSolverDirect(const RWPEBodyConstraintGraph* manager, const Vector3D<double> &gravity):
	RWPEConstraintSolver(manager,gravity)
{
}

RWPEConstraintSolverDirect::~RWPEConstraintSolverDirect() {
}

const RWPEConstraintSolver* RWPEConstraintSolverDirect::createSolver(const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity) const {
	return new RWPEConstraintSolverDirect(manager,gravity);
}

Eigen::VectorXd RWPEConstraintSolverDirect::solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
		Eigen::MatrixXd::Index constraintDim,
		const PropertyMap& pmap, RWPELogUtil* log) const
{
	const bool doLog = (log == NULL)? false : log->doLog();

	if (_manager == NULL)
		RW_THROW("RWPEConstraintSolverDirect (solve): There is no body-constraint manager set for this solver - please construct a new solver for body-constraint graph to use.");

	// First find the properties to use
	double PRECISION = pmap.get<double>(PROPERTY_SVD_PRECISION,-1.);

	if (PRECISION < 0) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (PRECISION < 0)
			PRECISION = tmpMap.get<double>(PROPERTY_SVD_PRECISION,-1.);
		RW_ASSERT(PRECISION > 0);
	}

	// Solution
	Eigen::VectorXd solution;
	RW_ASSERT(A.rows() == b.rows());
	if (A.rows() > 0) {
		solution = LinearAlgebra::pseudoInverse(A,PRECISION)*b;
		if (doLog) {
			std::ostream& lstr = log->log("Equation System",RWPE_LOCATION);
			lstr << "LHS: " << std::endl << A << std::endl;
			lstr << "RHS: " << b.transpose() << std::endl;

			const Eigen::VectorXd residual = A*solution-b;
			std::ostream& lstr2 = log->log("Solution",RWPE_LOCATION);
			lstr2 << "Solution: " << solution.transpose() << std::endl;
			lstr2 << "Residual: " << residual.transpose() << std::endl;
			lstr2 << "||Residual|| " << residual.norm() << std::endl;

			/*std::pair<Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<std::complex<double>, Eigen::Dynamic, -1> > dec = LinearAlgebra::eigenDecomposition(lhs);
			Eigen::VectorXd eigenValues(dec.second.rows());
			for (Eigen::VectorXd::Index i = 0; i < dec.second.rows(); i++) {
				const std::complex<double> &complex = dec.second(i,0);
				eigenValues[i] = complex.real();
			}
			RWPE_DEBUG_SOLVER("Eigenvalues: " << eigenValues.transpose() << ".");*/
		}
	} else
		solution = Eigen::VectorXd::Zero(0);
	return solution;
}

void RWPEConstraintSolverDirect::addDefaultProperties(PropertyMap& map) const {
	RWPEConstraintSolver::addDefaultProperties(map);
	map.add<double>(PROPERTY_SVD_PRECISION,"Precision of SVD - used for LinearAlgebra::pseudoInverse.",1e-6);
}
