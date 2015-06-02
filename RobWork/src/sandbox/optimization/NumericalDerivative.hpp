/*
 * NumericalDerivative.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: dagothar
 */

#ifndef SRC_RW_MATH_NUMERICALDERIVATIVE_HPP_
#define SRC_RW_MATH_NUMERICALDERIVATIVE_HPP_

#include <rw/math/Function.hpp>

namespace rw {
namespace math {

template<typename RES_T = double, typename ARG_T = double,
		typename GRAD_T = double>
class NumericalDerivative {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<NumericalDerivative> Ptr;

public:
	//! Constructor.
	NumericalDerivative() {
	}

	//! Destructor.
	virtual ~NumericalDerivative() {
	}

	/**
	 * @brief Get numerical derivative of a function at point x.
	 */
	virtual GRAD_T derivative(typename Function<RES_T, ARG_T>::Ptr function, ARG_T x) = 0;
};

} /* namespace math */
} /* namespace rw */

#endif /* SRC_RW_MATH_NUMERICALDERIVATIVE_HPP_ */
