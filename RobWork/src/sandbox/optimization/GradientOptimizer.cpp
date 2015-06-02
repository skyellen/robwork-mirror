/*
 * GradientOptimizer.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "FunctionWithNumericalDerivative.hpp"

#include "GradientOptimizer.hpp"

using namespace rw::common;
using namespace rw::math;

namespace rwlibs {
namespace optimization {

GradientOptimizer::GradientOptimizer(typename FunctionType::Ptr function,
		typename NumericalDerivativeType::Ptr numericalDerivative) :
		Optimizer(NULL), _numericalDerivative(numericalDerivative) {
	setFunction(function);
}

void GradientOptimizer::setFunction(typename FunctionType::Ptr function) {
	if (!dynamic_cast<Function1DiffType*>(function.get())) {
		RW_WARN(
				"GradientOptimizer received a function without derivative implementation. Wrapping function in numerical derivative object.");

		function = new FunctionWithNumericalDerivative<ResultType, VectorType,
				GradientType>(function, _numericalDerivative);

	}

	Optimizer::setFunction(function);
}

}
}

