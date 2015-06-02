/*
 * BFGSOptimizer.hpp
 *
 *  Created on: Feb 16, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_BFGSOPTIMIZER_HPP_
#define SRC_RWLIBS_OPTIMIZATION_BFGSOPTIMIZER_HPP_

#include "GradientOptimizer.hpp"
#include "GoldenSectionLineSearch.hpp"

namespace rwlibs {
namespace optimization {

/**
 * Implements Broyden-Fletcher-Goldfarb-Shanno optimization method.
 */
class BFGSOptimizer: public GradientOptimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<BFGSOptimizer> Ptr;

	//! Vector type used for internal calculations.
	typedef Eigen::VectorXd vector;

	//! Matrix type used for internal calculations.
	typedef Eigen::MatrixXd matrix;

public:
	/**
	 * @brief Constructor.
	 */
	BFGSOptimizer(typename FunctionType::Ptr function, double stepSize = 1.0,
			StopCondition::Ptr stopCondition = rw::common::ownedPtr(
					new StopCondition(100, 0.01)), LineSearch::Ptr strategy =
					rw::common::ownedPtr(new GoldenSectionLineSearch()));

	//! Destructor.
	virtual ~BFGSOptimizer();

	//! Set line search stop condition.
	void setLineSearchStopCondition(StopCondition::Ptr stopCondition) {
		_lineSearchStopCondition = stopCondition;
	}

	//! Set line search strategy.
	void setLineSearchStrategy(LineSearch::Ptr strategy);

protected:
	//! @copydoc Optimizer::newOptimization
	virtual void newOptimization(const VectorType& initialGuess,
			ResultType& initialValue, double& initialError);

	//! @copydoc Optimizer::step
	virtual void step(VectorType& currentGuess, ResultType& currentValue,
			double& currentError);

	//! Get line search strategy.
	LineSearch::Ptr getLineSearchStrategy() {
		return _lineSearch;
	}

	//! Get line search stop condition.
	StopCondition::Ptr getLineSearchStopCondition() {
		return _lineSearchStopCondition;
	}

	//! Access Hessian.
	matrix& getHessian() {
		return _B;
	}

	/**
	 * @brief Updates Hessian approximation using current step value and gradient difference.
	 *
	 * @param sk [in] step taken
	 * @param yk [in] gradient difference
	 */
	void updateHessian(const vector& sk, const vector& yk);

private:
	double _stepSize;
	LineSearch::Ptr _lineSearch;
	StopCondition::Ptr _lineSearchStopCondition;

	matrix _B; // inverse Hessian approximation
};

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_BFGSOPTIMIZER_HPP_ */
