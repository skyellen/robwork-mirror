/*
 * BFGSOptimizer.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: dagothar
 */

#include "BFGSOptimizer.hpp"

namespace rwlibs {
namespace optimization {

BFGSOptimizer::BFGSOptimizer(typename FunctionType::Ptr function,
		double stepSize, StopCondition::Ptr stopCondition,
		LineSearch::Ptr strategy) :
		GradientOptimizer(function), _stepSize(stepSize), _lineSearchStopCondition(
				stopCondition) {
	setLineSearchStrategy(strategy);
}

BFGSOptimizer::~BFGSOptimizer() {
}

void BFGSOptimizer::setLineSearchStrategy(LineSearch::Ptr strategy) {
	_lineSearch = strategy;
	_lineSearch->setFunction(getFunction());
	_lineSearch->setLeftBracket(0.0);
	_lineSearch->setRightBracket(_stepSize);
}

void BFGSOptimizer::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
	// initialize Hessian with identity matrix
	unsigned n = initialGuess.size();

	_B = matrix::Identity(n, n);
}

void BFGSOptimizer::step(VectorType& currentGuess, ResultType& currentValue,
		double& currentError) {
	//RW_WARN("1");
	/*
	 * Calculate a gradient at the current guess to pick the direction.
	 * Use Hessian approximation to alter the direction.
	 */
	Function1DiffType* f = dynamic_cast<Function1DiffType*>(getFunction().get());
	GradientType gradient0 = f->df(currentGuess);
	vector g0 = _B * -gradient0.e();
	GradientType direction = g0;

	//RW_WARN("2");
	/*
	 * Use line search to find the optimal step size.
	 */
	_lineSearch->setDirection(direction);
	VectorType newGuess = _lineSearch->optimize(currentGuess,
			*_lineSearchStopCondition);
	//VectorType newGuess = currentGuess - _stepSize * gradient0;

	//RW_WARN("3");
	/*
	 * Calculate gradient at the new point.
	 * Calculate gradient difference.
	 */
	GradientType gradient1 = f->df(newGuess);
	GradientType dGradient = gradient1 - gradient0;

	//RW_WARN("4");

	/*
	 * Update Hessian.
	 */
	vector sk = (newGuess - currentGuess).e();
	updateHessian(sk, dGradient.e());

	//RW_WARN("5");

	/*
	 * Update optimizer state.
	 */
	currentGuess = newGuess;
	currentError = gradient1.norm2();

	//RW_WARN("6");
}

void BFGSOptimizer::updateHessian(const vector& sk, const vector& yk) {
	RW_ASSERT(sk.size() == yk.size());

	unsigned n = sk.size(); // number of dimensions

	// calculate left temporary
	matrix leftTmp = matrix::Identity(n, n)
			- (sk * yk.transpose()) / (yk.transpose() * sk);

	// calculate right temporary
	matrix rightTmp = matrix::Identity(n, n)
			- (yk * sk.transpose()) / (yk.transpose() * sk);

	// calculate tail term
	matrix tailTmp = (sk * sk.transpose()) / (yk.transpose() * sk);

	// update Hessian
	_B = leftTmp * _B * rightTmp + tailTmp;
}

} /* namespace optimization */
} /* namespace rwlibs */
