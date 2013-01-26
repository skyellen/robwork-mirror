/*
 * NLLSIterationLog.cpp
 *
 *  Created on: Sep 20, 2012
 *      Author: bing
 */

#include "NLLSIterationLog.hpp"

namespace rwlibs {
namespace calibration {

NLLSIterationLog::NLLSIterationLog(int iterationNumber, double conditionNumber, bool isSingular, double residualNorm, double stepNorm, bool isConverged) :
		_iterationNumber(iterationNumber), _conditionNumber(conditionNumber), _isSingular(isSingular), _residualNorm(residualNorm), _stepNorm(stepNorm), _isConverged(isConverged) {

}

int NLLSIterationLog::getIterationNumber() const {
	return _iterationNumber;
}

double NLLSIterationLog::getConditionNumber() const {
	return _conditionNumber;
}

bool NLLSIterationLog::isSingular() const {
	return _isSingular;
}

double NLLSIterationLog::getResidualNorm() const {
	return _residualNorm;
}

double NLLSIterationLog::getStepNorm() const {
	return _stepNorm;
}

bool NLLSIterationLog::isConverged() const {
	return _isConverged;
}

}
}
