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

const double stepConvergenceTolerance = 1e-10;
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
	return (int)_iterationLogs.size();
}

NLLSIterationLog NLLSNewtonSolver::iterate() {
	RW_ASSERT(!_system.isNull());

	// Compute Jacobian.
	_system->computeJacobian(_jacobian);

	// Compute SVD of Jacobian.
	_jacobianSvd = _jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Compute residuals
	_system->computeResiduals(_residuals);

	//std::cout<<"Jacobian = "<<_jacobian.rows()<<" "<<_jacobian.cols()<<std::endl;
	//std::cout<<_jacobian<<std::endl;
	for (size_t i = 0; i<12; i++)
		std::cout<<"Residuals["<<i<<"] = "<<_residuals[i]<<std::endl;

	std::cout<<"||Redisuals|| = "<<_residuals.norm()<<std::endl;
	 
	// Compute step (solve Jacobian * step = residuals).
	_step = _jacobianSvd.solve(_residuals);
	//std::cout<<"ParameterCount = "<<_step.size()<<std::endl;
	// Apply step.
	_system->takeStep(_step);
	 
	// Log iteration.	
	const int iterationNumber = (int)_iterationLogs.size() + 1;
	const Eigen::VectorXd singularValues = _jacobianSvd.singularValues();
	const double conditionNumber = singularValues(singularValues.rows() - 1) / singularValues(0);
	std::cout<<"ConditionNumber = "<<conditionNumber<<std::endl;
	const bool isSingular = (singularValues.rows() != _jacobianSvd.nonzeroSingularValues());
	const double residualNorm = _residuals.norm();
	const double stepNorm = _step.norm();
	//std::cout<<"Step = "<<_step<<std::endl;	
	std::cout<<"Step Size= "<<stepNorm<<std::endl;
	_system->computeResiduals(_residuals);
	std::cout<<"||Redisuals After|| = "<<_residuals.norm()<<std::endl;
	const bool isConverged = _step.norm() <= stepConvergenceTolerance;
	NLLSIterationLog iterationLog(iterationNumber, conditionNumber, isSingular, residualNorm, stepNorm, isConverged);
	_iterationLogs.push_back(iterationLog);

	std::cout<<"Press enter to continue..."<<std::endl;
	char ch[4];
	std::cin.getline(ch, 1);

	// Verify iteration.
	if (isSingular)
		RW_THROW("Singular Jacobian.");
	if (conditionNumber < 1e-9) 
		RW_THROW("Condition number of "<<conditionNumber<<" indicates a singular set of equations.");
	if (boost::math::isnan(stepNorm))
		RW_THROW("NaN step.");
	if (boost::math::isinf(stepNorm))
		RW_THROW("Infinite step.");
	

	return iterationLog;
}

void NLLSNewtonSolver::solve() {
	while (true) {
		NLLSIterationLog iterationLog = iterate();
		//return;
		//std::cout << "Step: " << _step.transpose() << std::endl;
		//std::cout << "Iteration " << iterationLog.getIterationNumber() << " completed. Singular: " << (iterationLog.isSingular() ? "Yes" : "No")
		//	<< ". Condition: " << iterationLog.getConditionNumber() << ". ||Residuals||: " << iterationLog.getResidualNorm() << ". ||Step||: "
		//	<< iterationLog.getStepNorm() << "." << std::endl;

		// Stop iterating if converged.
		if (iterationLog.isConverged() )
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
