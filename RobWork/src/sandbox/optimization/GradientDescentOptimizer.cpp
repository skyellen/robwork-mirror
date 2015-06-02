/*
 * GradientDescentOptimizer.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "GradientDescentOptimizer.hpp"

namespace rwlibs {
namespace optimization {

GradientDescentOptimizer::GradientDescentOptimizer(
		typename FunctionType::Ptr function, double alpha, double stepSizeLimit) :
		GradientOptimizer(function), _alpha(alpha), _stepSizeLimit(
				stepSizeLimit) {
}

GradientDescentOptimizer::~GradientDescentOptimizer() {
}

void GradientDescentOptimizer::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
}

void GradientDescentOptimizer::step(VectorType& currentGuess,
		ResultType& currentValue, double& currentError) {
	// this is neccesary, because by default returned type is FunctionType::Ptr
	// we know that in this class this must be actually Function1DiffType::Ptr
	Function1DiffType* f = dynamic_cast<Function1DiffType*>(getFunction().get());

	/*
	 * 1. Calculate the gradient at the current guess.
	 */
	GradientType gradient = f->df(currentGuess);

	/*
	 * 2. Calculate a step to take based on the gradient.
	 */
	GradientType paramStep = -_alpha * gradient;

	/*
	 * 3. Modify the guess according to calculated step.
	 */
	currentGuess = currentGuess + paramStep;

	/*
	 * 4. Calculate new function value.
	 */
	currentValue = f->f(currentGuess);

	/*
	 * 5. Calculate current error
	 */
	currentError = gradient.norm2();
}

}
}

