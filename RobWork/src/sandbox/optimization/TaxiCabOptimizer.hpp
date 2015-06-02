/*
 * TaxiCabOptimizer.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_TAXICABOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_TAXICABOPTIMIZER_HPP_

#include "DirectionSetOptimizer.hpp"
#include "LineSearch.hpp"
#include "GoldenSectionLineSearch.hpp"

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

/**
 * Implements a taxi-cab method optimizer.
 *
 * Optimizer that searches sequentially through base directions using line search.
 */
class TaxiCabOptimizer: public DirectionSetOptimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<TaxiCabOptimizer> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to optimize
	 * @param stepSize [in] step size
	 * @param stopCondition [in] stop consition for the line search subroutine
	 * @param strategy [in] line search strategy (default: golden section)
	 */
	TaxiCabOptimizer(typename FunctionType::Ptr function, double stepSize = 1.0,
			StopCondition::Ptr stopCondition = rw::common::ownedPtr(
					new StopCondition(100, 0.01)), LineSearch::Ptr strategy =
					rw::common::ownedPtr(new GoldenSectionLineSearch()));

	//! Destructor.
	virtual ~TaxiCabOptimizer();

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

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_TAXICABOPTIMIZER_HPP_ */
