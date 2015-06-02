/*
 * StopCondition.hpp
 *
 *  Created on: Feb 11, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_STOPCONDITION_HPP_
#define SRC_RWLIBS_OPTIMIZATION_STOPCONDITION_HPP_

#include <limits>
#include "Optimizer.hpp"

namespace rwlibs {
namespace optimization {

// forward declaration of Optimizer
class Optimizer;

namespace {
typedef rw::common::Ptr<Optimizer> OptimizerPtr;
}

/** @addtogroup optimization */
/*@{*/

/**
 * Defines a stop condition for optimization algorithms.
 */
class StopCondition {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<StopCondition> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * Basic stop condition is a limit for the number of steps, and the lower
	 * limit for achieved tolerance. Condition is fulfilled when either one of these limits
	 * is exceeded.
	 *
	 * @param maxIterations [in] upper limit for iteration count (default: 100)
	 * @param minTolerance [in] lower limit for minimum error (default: 1e-3)
	 */
	StopCondition(unsigned long maxSteps = 100, double minTolerance = 0.001);

	//! Destructor.
	virtual ~StopCondition();

	/**
	 * @brief Is condition met?
	 *
	 * Checks optimizer state and returns \b true when condition is met.
	 */
	virtual bool check(OptimizerPtr optimizer) const;

	//! Get iteration limit.
	unsigned long getMaxSteps() const {
		return _maxSteps;
	}

	//! Get tolerance limit.
	double getMinTolerance() const {
		return _minTolerance;
	}

private:
	unsigned long _maxSteps;
	double _minTolerance;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_STOPCONDITION_HPP_ */
