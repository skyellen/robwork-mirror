/*
 * SimulatedAnnealingOptimizer.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#include "SimulatedAnnealingOptimizer.hpp"

#include <rw/math/Math.hpp>

using namespace rw::math;

namespace rwlibs {
namespace optimization {

SimulatedAnnealingOptimizer::SimulatedAnnealingOptimizer(
		typename FunctionType::Ptr function, double dev, double T, double dT) :
		Optimizer(function), _deviation(dev), _initialT(T), _dT(dT) {
}

SimulatedAnnealingOptimizer::~SimulatedAnnealingOptimizer() {
}

void SimulatedAnnealingOptimizer::newOptimization(
		const VectorType& initialGuess, ResultType& initialValue,
		double& initialError) {
	_T = _initialT;

	// calculate initial value and error
	FunctionType::Ptr f = getFunction();
	initialValue = f->f(initialGuess);
	//initialError = _T;
}

void SimulatedAnnealingOptimizer::step(VectorType& currentGuess,
		ResultType& currentValue, double& currentError) {
	/*
	 * Pick a random neighbour
	 */
	VectorType previousGuess = currentGuess;
	// ResultType previousValue = currentValue;  // not used
	VectorType newState = generateNewState(previousGuess,
			_deviation * (_T / _initialT));

	/*
	 * Evaluate states
	 */
	FunctionType::Ptr f = getFunction();
	ResultType newValue = f->f(newState);

	/*
	 * Choose whether to accept new state
	 */
	if (rw::math::Math::ran(0.0, 1.0)
			< calculateAcceptanceProbability(currentValue, newValue, _T)) {
		currentGuess = newState;
		currentValue = newValue;

		/*
		 * Calculate tolerance
		 */
//		double dValue = fabs(previousValue - currentValue);
//		double distance = (currentGuess - previousGuess).norm2();
//		currentError = dValue / distance;
	}

	/*
	 * Update temperature
	 */
	_T -= _dT;
	if (_T < 0.0) {
		_T = 0.0;
	}

	currentError = _T;
}

VectorType SimulatedAnnealingOptimizer::generateNewState(
		const VectorType& currentState, double deviation) {
	VectorType newState = currentState;

	newState += Math::ranDir(newState.size(), Math::ran(0.0, deviation));

	return newState;
}

double SimulatedAnnealingOptimizer::calculateAcceptanceProbability(
		ResultType e0, ResultType e1, double T) {
	if (e1 < e0) {
		return 1.0;
	} else if (T == 0.0) {
		return 0.0;
	} else {
		return exp(-fabs(e0 - e1) / T);
	}
}

} /* namespace optimization */
} /* namespace rwlibs */
