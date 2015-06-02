/*
 * GradientDescentOptimizer.hpp
 *
 *  Created on: Feb 7, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_GRADIENTDESCENTOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_GRADIENTDESCENTOPTIMIZER_HPP_

#include "GradientOptimizer.hpp"
#include "Types.hpp"

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

/**
 * @brief Implements simple gradient descent algorithm.
 *
 * Implements optimization for functions of type: f(VectorType) -> double.
 * Requires functions which are 1-time differentiable.
 *
 * Each step, the current guess is modified according to the direction
 * of gradient multiplied by the learning coefficient \b alpha.
 */
class GradientDescentOptimizer: public GradientOptimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<GradientDescentOptimizer> Ptr;

public:
	/**
	 * @brief Constructor
	 *
	 * @param function [in] a function with a first order derivative
	 * @param alpha [in] the learning coefficient (default: 1.0)
	 * @param stepSizeLimit [in] the limit for the step size (default: std::numeric_limits<double>::max())
	 */
	GradientDescentOptimizer(typename FunctionType::Ptr function, double alpha =
			1.0, double stepSizeLimit = std::numeric_limits<double>::max());

	//! Destructor.
	virtual ~GradientDescentOptimizer();

	//! Get learning coefficient.
	double getAlpha() const {
		return _alpha;
	}

	//! Set learning coefficient.
	void setAlpha(double alpha) {
		_alpha = alpha;
	}

	//! Get the limit for step size
	double getStepSizeLimit() const {
		return _stepSizeLimit;
	}

	//! Set the limit for step size
	void setStepSizeLimit(double limit) {
		_stepSizeLimit = limit;
	}

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

private:
	double _alpha;
	double _stepSizeLimit;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_GRADIENTDESCENTOPTIMIZER_HPP_ */
