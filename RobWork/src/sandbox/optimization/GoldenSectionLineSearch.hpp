/*
 * GoldenSectionLineSearch.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_GOLDENSECTIONLINESEARCH_HPP_
#define SRC_RWLIBS_OPTIMIZATION_GOLDENSECTIONLINESEARCH_HPP_

#include "LineSearch.hpp"

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

/**
 * Implements line search using golden-section ratio method.
 */
class GoldenSectionLineSearch: public LineSearch {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<GoldenSectionLineSearch> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * Default constructor for passing as a strategy argument.
	 */
	GoldenSectionLineSearch();

	/**
	 * @brief Constructor.
	 */
	GoldenSectionLineSearch(typename FunctionType::Ptr function,
			VectorType direction, double leftBracket = -1.0,
			double rightBracket = 1.0);

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

private:
	// current bracket values
	double _a;
	double _b;

	// values of function at the bracket points
	double _leftValue;
	double _rightValue;

	// for calculating tolerance
	VectorType _previousGuess;
	ResultType _previousValue;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_GOLDENSECTIONLINESEARCH_HPP_ */
