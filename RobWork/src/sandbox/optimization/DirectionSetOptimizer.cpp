/*
 * DirectionSetOptimizer.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#include "DirectionSetOptimizer.hpp"

namespace rwlibs {
namespace optimization {

DirectionSetOptimizer::DirectionSetOptimizer(
		typename FunctionType::Ptr function, double stepSize,
		StopCondition::Ptr stopCondition, LineSearch::Ptr strategy) :
		Optimizer(function), _stepSize(stepSize), _lineSearchStopCondition(
				stopCondition) {
	setLineSearchStrategy(strategy);
}

DirectionSetOptimizer::~DirectionSetOptimizer() {
}

void DirectionSetOptimizer::setLineSearchStrategy(LineSearch::Ptr strategy) {
	_lineSearch = strategy;
	_lineSearch->setFunction(getFunction());
	_lineSearch->setLeftBracket(-_stepSize);
	_lineSearch->setRightBracket(_stepSize);
}

} /* namespace optimization */
} /* namespace rwlibs */
