/*
 * Optimizer.hpp
 *
 *  Created on: Feb 7, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_OPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_OPTIMIZER_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Function.hpp>
//#include <rwlibs/algorithms/IteratedAlgorithm.hpp>
#include "StopCondition.hpp"
#include "Types.hpp"

namespace rwlibs {
namespace optimization {

// forward declaration of StopCondition
class StopCondition;

/** @addtogroup optimization */
/*@{*/

/**
 * @brief Base class for function optimizers.
 *
 * Finds minima of Multi-Input Single-Output functions.
 */
class Optimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<Optimizer> Ptr;

	//! Optimizer state data structure for logging.
	struct State {
		unsigned long step;
		VectorType guess;
		ResultType value;
		double error;
	};

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to minimize
	 */
	Optimizer(typename FunctionType::Ptr function);

	//! Destructor.
	virtual ~Optimizer();

	//! Get optimized function.
	typename FunctionType::Ptr getFunction() const {
		return _function;
	}

	//! Set optimized function
	virtual void setFunction(typename FunctionType::Ptr function) {
		_function = function;
	}

	/**
	 * @brief Perform optimization.
	 *
	 * Resets iteration count, tolerance & clears log.
	 *
	 * @param initialGuess [in] initial guess for the function minimum
	 * @param stopCondition [in] stop condition for the algorithm
	 *
	 * @return Optimization result.
	 */
	VectorType optimize(VectorType initialGuess,
			const StopCondition& stopCondition);

	/**
	 * @brief Get current result.
	 */
	VectorType getCurrentGuess() const {
		return _currentGuess;
	}

	/**
	 * @brief Get function value at the current result
	 */
	ResultType getCurrentValue() const {
		return _currentValue;
	}

	//! Get number of steps taken.
	unsigned long getStepCount() const {
		return _stepCount;
	}

	//! Get current tolerance.
	double getCurrentError() const {
		return _currentError;
	}

	/**
	 * @brief Enable/disable logging.
	 *
	 * Optimizer logs current iteration number, result, and error in a vector structure.
	 */
	void setLogging(bool enable) {
		_logging = enable;
	}

	/**
	 * @brief Access optimization log.
	 */
	const std::vector<State>& getLog() const {
		return _log;
	}

	//! Clear optimization log.
	void clearLog() {
		_log.clear();
	}

protected:
	/**
	 * @brief Initialization of the algorithm for the new optimization.
	 */
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError) = 0;

	/**
	 * @brief Implements a step of the algorithm.
	 *
	 * @param currentGuess [out] next guess after the step has been taken
	 * @param currentValue [out] value of function at the next guess
	 * @param currentError [out] error after the step has been taken
	 */
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError) = 0;

private:
	typename FunctionType::Ptr _function;

	VectorType _currentGuess;
	ResultType _currentValue;

	unsigned long _stepCount;
	double _currentError;

	bool _logging;
	std::vector<State> _log;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_OPTIMIZER_HPP_ */
