/*
 * PowellOptimizer.hpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_POWELLOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_POWELLOPTIMIZER_HPP_

#include "DirectionSetOptimizer.hpp"

namespace rwlibs {
namespace optimization {

/**
 * Implements Powell direction set based optimization method.
 */
class PowellOptimizer: public DirectionSetOptimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<PowellOptimizer> Ptr;

public:
	/**
	 * @brief Constructor.
	 */
	PowellOptimizer(typename FunctionType::Ptr function, double stepSize = 1.0,
			StopCondition::Ptr stopCondition = rw::common::ownedPtr(
					new StopCondition(100, 0.01)), LineSearch::Ptr strategy =
					rw::common::ownedPtr(new GoldenSectionLineSearch()));

	//! Destructor.
	virtual ~PowellOptimizer();

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

private:
	// which direction do we use next step?
	unsigned _dirIdx;

	// previous result (for calculating tolerance)
	VectorType _previousGuess;
	ResultType _previousValue;
};

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_POWELLOPTIMIZER_HPP_ */
