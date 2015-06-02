/*
 * TaxiCabOptimizer.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: dagothar
 */

#include "TaxiCabOptimizer.hpp"

#include <iostream>

using namespace rw::math;

namespace rwlibs {
namespace optimization {

TaxiCabOptimizer::TaxiCabOptimizer(typename FunctionType::Ptr function,
		double stepSize, StopCondition::Ptr stopCondition,
		LineSearch::Ptr strategy) :
		DirectionSetOptimizer(function, stepSize, stopCondition, strategy), _dirIdx(
				0) {

}

TaxiCabOptimizer::~TaxiCabOptimizer() {
}

void TaxiCabOptimizer::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
	/*
	 * Build directions set from unit vectors.
	 */

	unsigned dims = initialGuess.size();

	DirectionSet& directionSet = getDirectionSet();
	directionSet.clear();

	for (unsigned i = 0; i < dims; ++i) {
		Q unit(dims, 0.0);
		unit(i) = 1.0;

		directionSet.push_back(std::make_pair(unit, 0.0));
	}

	typename FunctionType::Ptr f = getFunction();
	initialValue = f->f(initialGuess);

	_dirIdx = 0;
}

void TaxiCabOptimizer::step(VectorType& currentGuess, ResultType& currentValue,
		double& currentError) {
	/*
	 * Perform line search along the current dimension.
	 */
	//RW_WARN("1");
	LineSearch::Ptr lineSearch = getLineSearchStrategy();
	StopCondition::Ptr stopCondition = getLineSearchStopCondition();
	//RW_WARN("2");

	DirectionSet& directionSet = getDirectionSet();
	//std::cout << "Direction: " << directionSet[_dirIdx].first << std::endl;
	lineSearch->setDirection(directionSet[_dirIdx].first);
	//RW_WARN("3");

	currentGuess = lineSearch->optimize(currentGuess, *stopCondition);
	currentValue = lineSearch->getCurrentValue();
	//RW_WARN("4");

	// update direction index
	++_dirIdx;

	if (_dirIdx >= directionSet.size()) {
		ResultType dValue = currentValue - _previousValue;
		double distance = (currentGuess - _previousGuess).norm2();
		currentError = fabs(dValue / distance);

		_previousGuess = currentGuess;
		_previousValue = currentValue;

		_dirIdx = 0;
	}
	//RW_WARN("5");
}

} /* namespace optimization */
} /* namespace rwlibs */
