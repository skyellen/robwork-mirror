/*
 * Function1DiffFirstDifferences.hpp
 *
 *  Created on: Feb 8, 2015
 *      Author: dagothar
 */

#ifndef SRC_RW_MATH_FIRSTDIFFERENCES_HPP_
#define SRC_RW_MATH_FIRSTDIFFERENCES_HPP_

#include "NumericalDerivative.hpp"

namespace rw {
namespace math {

// TODO use SFINAE to concept check types of gradients
// should make matrices for MIMO functions, vectors for MISO etc
namespace { /* anonymous namespace */
/**
 * @brief Calculate first differences for functions of type
 * f: Vector_type -> double
 *
 * GRAD_T should be a vector type, which provides constructor initialization based on size,
 * and norm2() that returns its norm.
 */
template<typename RES_T, typename ARG_T, typename GRAD_T>
GRAD_T doFirstDifferences(typename Function<RES_T, ARG_T>::Ptr func, ARG_T q,
		double step) {
	GRAD_T diffs(q.size());

	for (unsigned i = 0; i < q.size(); ++i) {
		// step only for one dimension
		ARG_T step1(q.size(), 0.0);
		step1(i) = step;

		diffs(i) = (1.0 / step)
				* (func->f(q + 0.5 * step1) - func->f(q - 0.5 * step1));
	}

	return diffs;
}

/**
 * @brief Calculate first differences for functions of type
 * f: double -> double
 */
template<>
double doFirstDifferences<double, double, double>(typename Function<>::Ptr func,
		double q, double step) {
	return (1.0 / step) * (func->f(q + 0.5 * step) - func->f(q - 0.5 * step));
}

} /* anonymous namespace */

/** @addtogroup math */
/*@{*/

/**
 * Implements numeric differentiation using first differences
 * for functions of type f: double/Vector_type -> double.
 *
 * @note The issue here is that multidimensional functions return
 * matrix-type gradient, and template specialization is difficult in that case.
 */
template<typename RES_T = double, typename ARG_T = double,
		typename GRAD_T = double>
class FirstDifferences: public NumericalDerivative<RES_T, ARG_T, GRAD_T> {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<FirstDifferences> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to wrap
	 * @param step [in] step size for calculating first differences (default: 1.0)
	 */
	FirstDifferences(double step = 1.0) :
			_step(step) {
	}

	/**
	 * @brief Calculates gradient using first differences.
	 */
	virtual GRAD_T derivative(typename Function<RES_T, ARG_T>::Ptr function,
			ARG_T x) {
		GRAD_T gradient = doFirstDifferences<RES_T, ARG_T, GRAD_T>(function, x,
				_step);

		return gradient;
	}

	double getStepSize() const {
		return _step;
	}

	void setStepSize(double step) {
		_step = step;
	}

private:
	double _step;
};

/*@}*/

} /* namespace math */
} /* namespace rw */

#endif /* SRC_RW_MATH_FIRSTDIFFERENCES_HPP_ */
