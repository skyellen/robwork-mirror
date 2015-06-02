/*
 * GradientOptimizer.hpp
 *
 *  Created on: Feb 9, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_GRADIENTOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_GRADIENTOPTIMIZER_HPP_

#include <rw/math/NumericalDerivative.hpp>
#include <rw/math/FirstDifferences.hpp>
#include "Optimizer.hpp"
#include "Types.hpp"

namespace rwlibs {
namespace optimization {

namespace {
//! Numerical derivative method type
typedef typename rw::math::NumericalDerivative<ResultType, VectorType,
		GradientType> NumericalDerivativeType;

//! Numerical derivative first differences  method type
typedef typename rw::math::FirstDifferences<ResultType, VectorType, GradientType> FirstDifferencesMethod;
}

/** @addtogroup optimization */
/*@{*/

/**
 * Base class for gradient based optimizers.
 *
 * Implements optimization for functions of type: f(VectorType) -> double.
 * Requires functions which are 1-time differentiable.
 * If the provided function does not provide a derivative,
 * the gradient optimizer will wrap it in with numerical derivative.
 */
class GradientOptimizer: public Optimizer {
public:
//! Smart pointer.
	typedef rw::common::Ptr<GradientOptimizer> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to optimize.
	 * @param numericalDerivative [in] a preferred method for wrapping functions with numerical derivatives (default: uses first differences)
	 */
	GradientOptimizer(typename FunctionType::Ptr function,
			typename NumericalDerivativeType::Ptr numericalDerivative =
					rw::common::ownedPtr(new FirstDifferencesMethod(0.01)));

	/**
	 * @copydoc OptimizerBase::setFunction
	 *
	 * This implementation makes sure we receive a differentiable function.
	 * If a function without derivative implementation is passed, it is automatically wrapped with
	 * a numerical derivative.
	 */
	virtual void setFunction(typename FunctionType::Ptr function);

private:
	typename NumericalDerivativeType::Ptr _numericalDerivative;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_GRADIENTOPTIMIZER_HPP_ */
