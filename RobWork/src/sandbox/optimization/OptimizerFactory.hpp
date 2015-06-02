/*
 * OptimizerFactory.hpp
 *
 *  Created on: Feb 11, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_OPTIMIZERFACTORY_HPP_
#define SRC_RWLIBS_OPTIMIZATION_OPTIMIZERFACTORY_HPP_

#include "Optimizer.hpp"
#include "LineSearch.hpp"

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

class OptimizerFactory {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<OptimizerFactory> Ptr;

public:
	/**
	 * @brief Create a basic all-round optimizer.
	 */
	virtual Optimizer::Ptr makeOptimizer(typename FunctionType::Ptr function);

	/**
	 * @brief Create a direct optimizer.
	 *
	 * Direct optimizers use only function value evaluations, and thus doesn't require
	 * function with derivatives.
	 */
	virtual Optimizer::Ptr makeDirectOptimizer(
			typename FunctionType::Ptr function);

	/**
	 * @brief Create a gradient optimizer.
	 *
	 * Gradient optimizers require functions having a derivative.
	 */
	virtual Optimizer::Ptr makeGradientOptimizer(
			typename FunctionType::Ptr function);

	/**
	 * @brief Creates an all-round line search optimizer.
	 */
	virtual LineSearch::Ptr makeLineSearch(typename FunctionType::Ptr function,
			VectorType direction, double leftBracket = -1.0,
			double rightBracket = 1.0);
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_OPTIMIZERFACTORY_HPP_ */
