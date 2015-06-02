/*
 * LineSearch.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_LINESEARCH_HPP_
#define SRC_RWLIBS_OPTIMIZATION_LINESEARCH_HPP_

#include "Optimizer.hpp"
#include "Types.hpp"

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

/**
 * Interface for line search methods.
 *
 * Finds minimum of function f along specified line.
 */
class LineSearch: public Optimizer {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<LineSearch> Ptr;

public:
	/**
	 * @brief Default constructor.
	 *
	 * Useful when passing as an argument to optimizers to indicate line search strategy.
	 */
	LineSearch();

	/**
	 * @brief Constructor.
	 */
	LineSearch(typename FunctionType::Ptr function, VectorType direction,
			double leftBracket = -1.0, double rightBracket = 1.0);

	//! Get search direction.
	const VectorType& getDirection() const {
		return _direction;
	}

	//! Set search direction.
	void setDirection(const VectorType& direction) {
		_direction = direction / direction.norm2();
	}

	double getLeftBracket() const {
		return _leftBracket;
	}

	void setLeftBracket(double leftBracket) {
		_leftBracket = leftBracket;
	}

	double getRightBracket() const {
		return _rightBracket;
	}

	void setRightBracket(double rightBracket) {
		_rightBracket = rightBracket;
	}

	VectorType getStartPoint() const {
		return _startPoint;
	}

	void setStartPoint(VectorType startPoint) {
		_startPoint = startPoint;
	}

protected:
	/**
	 * @brief Based on the interval parameter t, return Q argument for function evaluation.
	 *
	 * @param startPoint [in] start point of the line
	 * @param direction [in] direction of the line (vector of unit length)
	 * @param t [in] linear parameter
	 */
	VectorType getParameterAt(double t) const {
		return _startPoint + t * (_direction / _direction.norm2());
	}

private:
	VectorType _startPoint;
	VectorType _direction;
	double _leftBracket;
	double _rightBracket;
};

/*@}*/

} /* namespace optimization */
} /* namespace rwlibs */

#endif /* SRC_RWLIBS_OPTIMIZATION_LINESEARCH_HPP_ */
