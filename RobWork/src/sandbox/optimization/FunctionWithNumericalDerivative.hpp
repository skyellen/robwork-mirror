/*
 * Function1DiffNumeric.hpp
 *
 *  Created on: Feb 8, 2015
 *      Author: dagothar
 */

#ifndef SRC_RW_MATH_FUNCTIONWITHNUMERICALDERIVATIVE_
#define SRC_RW_MATH_FUNCTIONWITHNUMERICALDERIVATIVE_

#include <rw/common/Ptr.hpp>
#include <rw/math/Function.hpp>
#include "NumericalDerivative.hpp"

namespace rw {
namespace math {

/** @addtogroup math */
/*@{*/

/**
 * Interface to adapt a function without derivative into function with numerically calculated derivative
 */
template<typename RES_T = double, typename ARG_T = double, typename GRAD_T = double>
class FunctionWithNumericalDerivative: public rw::math::Function1Diff<RES_T,
		ARG_T, GRAD_T> {
public:
	//! Smart pointer.
	typedef rw::common::Ptr<FunctionWithNumericalDerivative> Ptr;

public:
	/**
	 * @brief Constructor.
	 *
	 * @param function [in] a function to wrap
	 */
	FunctionWithNumericalDerivative(
			typename Function<RES_T, ARG_T>::Ptr function,
			typename NumericalDerivative<RES_T, ARG_T, GRAD_T>::Ptr derivative) :
			_function(function), _derivative(derivative) {
	}

	//! Destructor.
	virtual ~FunctionWithNumericalDerivative() {
	}

	/**
	 * @copydoc Function::f
	 *
	 * Forwards call to the wrapped function object.
	 */
	virtual RES_T f(ARG_T q) {
		return _function->f(q);
	}

	/**
	 * @brief Returns numerically calculated function derivative
	 */
	virtual GRAD_T df(ARG_T q) {
		return _derivative->derivative(_function, q);
	}

	//! Get wrapped function.
	typename Function<RES_T, ARG_T>::Ptr getFunction() {
		return _function;
	}

private:
	typename Function<RES_T, ARG_T>::Ptr _function;
	typename NumericalDerivative<RES_T, ARG_T, GRAD_T>::Ptr _derivative;
};

/*@}*/

} /* namespace math */
} /* namespace rw */

#endif /* SRC_RW_MATH_FUNCTIONWITHNUMERICALDERIVATIVE_ */
