/*
 * NLLSNewtonSolver.cpp
 *
 *  Created on: Sep 12, 2012
 *      Author: bing
 */

#include "NLLSNewtonSolver.hpp"

#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/SVD>
#include <rw/common.hpp>

namespace rwlibs {
namespace calibration {

const double stepConvergenceTolerance = 1e-14;
const int maxIterationCount = 100;

NLLSNewtonSolver::NLLSNewtonSolver(NLLSSystem::Ptr system) :
		_system(system) {

}

NLLSNewtonSolver::~NLLSNewtonSolver() {

}

NLLSSystem::Ptr NLLSNewtonSolver::getSystem() const {
	return _system;
}

const std::vector<NLLSIterationLog>& NLLSNewtonSolver::getIterationLogs() const {
	return _iterationLogs;
}

const int NLLSNewtonSolver::getIterationCount() const {
	return _iterationLogs.size();
}

NLLSIterationLog NLLSNewtonSolver::iterate() {
	RW_ASSERT(!_system.isNull());

	// Compute Jacobian.
	_system->computeJacobian(_jacobian);

	// Compute SVD of Jacobian.
	_jacobianSvd = _jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Compute residuals
	_system->computeResiduals(_residuals);

	// Compute step (solve Jacobian * step = residuals).
	_step = _jacobianSvd.solve(-_residuals);

	// Apply step.
	_system->takeStep(_step);
	
	// Log iteration.	
	const int iterationNumber = _iterationLogs.size() + 1;
	const Eigen::VectorXd singularValues = _jacobianSvd.singularValues();
	const double conditionNumber = singularValues(0) / singularValues(singularValues.rows() - 1);
	const bool isSingular = (singularValues.rows() != _jacobianSvd.nonzeroSingularValues());
	const double residualNorm = _residuals.norm();
	const double stepNorm = _step.norm();
	const bool isConverged = _step.norm() <= stepConvergenceTolerance;
	NLLSIterationLog iterationLog(iterationNumber, conditionNumber, isSingular, residualNorm, stepNorm, isConverged);
	_iterationLogs.push_back(iterationLog);

	// Verify iteration.
	if (isSingular)
		RW_THROW("Singular Jacobian.");
	if (boost::math::isnan(stepNorm))
		RW_THROW("NaN step.");
	if (boost::math::isinf(stepNorm))
		RW_THROW("Infinite step.");

	return iterationLog;
}

void NLLSNewtonSolver::solve() {
	while (true) {
		NLLSIterationLog iterationLog = iterate();

		//std::cout << "Step: " << _step.transpose() << std::endl;
		//std::cout << "Iteration " << iterationLog.getIterationNumber() << " completed. Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
		//	<< ". Condition: " << iterationLog.getConditionNumber() << ". ||Residuals||: " << iterationLog.getResidualNorm() << ". ||Step||: "
		//	<< iterationLog.getStepNorm() << "." << std::endl;

		// Stop iterating if converged.
		if (iterationLog.isConverged())
			break;

		// Throw exception if iteration limit has been reached.
		if (maxIterationCount > 0 && iterationLog.getIterationNumber() >= maxIterationCount)
			RW_THROW("Iteration limit reached.");
	}
}

Eigen::MatrixXd NLLSNewtonSolver::estimateCovarianceMatrix() const {
	RW_ASSERT(_jacobianSvd.rows() != 0 && _jacobianSvd.cols() != 0);

	// Eq. 15.4.20 from Numerical Recipes (covariance matrix of unknown variables)
	const Eigen::MatrixXd V = _jacobianSvd.matrixV();
	const Eigen::VectorXd singularValues = _jacobianSvd.singularValues();

	const double eps = std::numeric_limits<double>::epsilon();
	const double precision = eps * _jacobianSvd.rows() * singularValues(0);

	Eigen::MatrixXd covarianceMatrix(V.rows(), V.cols());
	for (int j = 0; j < V.rows(); j++) {
		for (int k = 0; k < V.rows(); k++) {
			double sum = 0;
			for (int i = 0; i < _jacobianSvd.cols(); i++) {
				if (singularValues(i) > precision)
					sum += (V(j, i) * V(k, i)) / (singularValues(i) * singularValues(i));
			}
			covarianceMatrix(j, k) = sum;
		}
	}

	return covarianceMatrix;
}

}
}
