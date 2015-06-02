/*
 * GoldenSectionLineSearch.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "GoldenSectionLineSearch.hpp"

#include <iostream>

using namespace rw::math;

namespace rwlibs {
namespace optimization {

GoldenSectionLineSearch::GoldenSectionLineSearch() {
}

GoldenSectionLineSearch::GoldenSectionLineSearch(
		typename FunctionType::Ptr function, VectorType direction,
		double leftBracket, double rightBracket) :
		LineSearch(function, direction, leftBracket, rightBracket) {
}

void GoldenSectionLineSearch::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
	// set line start point as the initial guess
	setStartPoint(initialGuess);

	// calculate values at the ends of the bracket
	typename FunctionType::Ptr f = getFunction();

	_a = getLeftBracket();
	_b = getRightBracket();

	_leftValue = f->f(getParameterAt(_a));
	_rightValue = f->f(getParameterAt(_b));

	_previousGuess = initialGuess;
	_previousValue = f->f(initialGuess);
}

void GoldenSectionLineSearch::step(VectorType& currentGuess, ResultType& currentValue,
		double& currentError) {
	const double Phi = 0.618;

	typename FunctionType::Ptr f = getFunction();

	/*
	 * 1. Calculate values at midpoints c & d.
	 */
	double c = _b + Phi * (_a - _b);
	double d = _a + Phi * (_b - _a);

	double cValue = f->f(getParameterAt(c));
	double dValue = f->f(getParameterAt(d));

	/*
	 * 2. Pick a new interval.
	 */
	if (cValue < dValue) {
		// move b -> d
		_b = d;
		_rightValue = dValue;
	} else {
		// move a -> c
		_a = c;
		_leftValue = cValue;
	}

	// update optimizer
	currentGuess = getParameterAt((_a + _b) / 2.0);
	currentValue = f->f(currentGuess);
	currentError = fabs(currentValue - _previousValue)
			/ (currentGuess - _previousGuess).norm2();

	_previousGuess = currentGuess;
	_previousValue = currentValue;
}

} /* namespace optimization */
} /* namespace rwlibs */
