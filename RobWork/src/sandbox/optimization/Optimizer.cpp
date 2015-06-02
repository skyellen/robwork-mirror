/*
 * Optimizer.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "Optimizer.hpp"

namespace rwlibs {
namespace optimization {

Optimizer::Optimizer(typename FunctionType::Ptr function) :
		_function(function), _currentGuess(VectorType()), _currentValue(
				ResultType()), _stepCount(0), _currentError(
				std::numeric_limits<double>::max()), _logging(false) {
}

Optimizer::~Optimizer() {
}

VectorType Optimizer::optimize(VectorType initialGuess,
		const StopCondition& stopCondition) {

	// reset iteration count, error, and clear log
	_stepCount = 0;
	_currentError = std::numeric_limits<double>::max();
	clearLog();

	// perform algorithm initialization
	_currentGuess = initialGuess;
	newOptimization(initialGuess, _currentValue, _currentError);

	// take algorithm steps...
	while (!stopCondition.check(this)) {

		// actual step
		step(_currentGuess, _currentValue, _currentError);

		// log step
		if (_logging) {
			State state = { _stepCount, _currentGuess, _currentValue,
					_currentError };
			_log.push_back(state);
		}

		++_stepCount;
	}

	return _currentGuess;
}

}
}

