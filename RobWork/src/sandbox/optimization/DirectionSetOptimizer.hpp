/*
 * DirectionSetOptimizer.hpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_DIRECTIONSETOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_DIRECTIONSETOPTIMIZER_HPP_

#include "Optimizer.hpp"
#include "GoldenSectionLineSearch.hpp"

namespace rwlibs {
namespace optimization {

/**
 * Base class for optimization methods using a direction set approach.
 */
class DirectionSetOptimizer: public Optimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<DirectionSetOptimizer> Ptr;

	/**
	 * @brief A type for a direction used by direction set methods.
	 * Holds a direction and a quality measure of that direction.
	 */
	typedef std::pair<VectorType, double> Direction;

	//! A type for a direction set.
	typedef std::vector<Direction> DirectionSet;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to optimize
	 * @param stepSize [in] step size limit
	 * @param stopCondition [in] stop condition for the line search subroutine
	 * @param strategy [in] line search strategy (default: golden section)
	 */
	DirectionSetOptimizer(typename FunctionType::Ptr function, double stepSize =
			1.0,
			StopCondition::Ptr stopCondition = rw::common::ownedPtr(
					new StopCondition(100, 0.01)), LineSearch::Ptr strategy =
					rw::common::ownedPtr(new GoldenSectionLineSearch()));

	//! Destructor.
	virtual ~DirectionSetOptimizer();

	//! Get step size.
	double getStepSize() const {
		return _stepSize;
	}

	//! Set step size.
	void setStepSize(double stepSize) {
		RW_ASSERT(stepSize > 0.0);

		_stepSize = stepSize;
	}

	//! Set line search stop condition.
	void setLineSearchStopCondition(StopCondition::Ptr stopCondition) {
		_lineSearchStopCondition = stopCondition;
	}

	//! Set line search strategy.
	void setLineSearchStrategy(LineSearch::Ptr strategy);

protected:
	//! Get line search strategy.
	LineSearch::Ptr getLineSearchStrategy() {
		return _lineSearch;
	}

	//! Get line search stop condition.
	StopCondition::Ptr getLineSearchStopCondition() {
		return _lineSearchStopCondition;
	}

	//! Get access to the direction set.
	DirectionSet& getDirectionSet() {
		return _directionSet;
	}

private:
	double _stepSize;
	LineSearch::Ptr _lineSearch;
	StopCondition::Ptr _lineSearchStopCondition;
	DirectionSet _directionSet;
};

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_DIRECTIONSETOPTIMIZER_HPP_ */
