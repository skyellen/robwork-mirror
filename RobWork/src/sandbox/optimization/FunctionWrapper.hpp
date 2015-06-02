/*
 * FunctionWrapper.hpp
 *
 *  Created on: Feb 7, 2015
 *      Author: dagothar
 */

#ifndef SRC_RW_MATH_FUNCTIONWRAPPER_HPP_
#define SRC_RW_MATH_FUNCTIONWRAPPER_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Function.hpp>
#include <boost/function.hpp>

namespace rw {
namespace math {

/** @addtogroup math */
/*@{*/

/**
 * A wrapper to create a Function from a callback
 */
template<class RES_T = double, class ARG_T = double>
class FunctionWrapper: virtual public Function<RES_T, ARG_T> {
public:
	//! Smart pointer type
	typedef rw::common::Ptr<FunctionWrapper> Ptr;

public:
	/**
	 * @brief Constructor
	 *
	 * @param f [in] a callback that returns the function value
	 */
	FunctionWrapper(typename boost::function<RES_T(ARG_T)> f) :
			_f(f) {
	}

	//! Destructor
	virtual ~FunctionWrapper() {
	}

	/**
	 * @copydoc Function::f
	 *
	 * Redirects function call to the wrapped callback.
	 */
	virtual RES_T f(ARG_T q) {
		return _f(q);
	}

	//! Get the wrapped callback.
	typename boost::function<RES_T(ARG_T)> getCallback() {
		return _f;
	}

private:
	typename boost::function<RES_T(ARG_T)> _f;
};

/**
 * A wrapper to create a Function1Diff from a callback.
 */
template<class RES_T = double, class ARG_T = double, class GRAD_T = double>
class Function1DiffWrapper: public FunctionWrapper<RES_T, ARG_T>,
		virtual public Function1Diff<RES_T, ARG_T, GRAD_T> {
public:
	//! Smart pointer type
	typedef rw::common::Ptr<Function1DiffWrapper> Ptr;

public:
	/**
	 * @brief Constructor
	 *
	 * @param f [in] a callback that returns the function value
	 * @param df [in] a callback that returns the function derivative
	 */
	Function1DiffWrapper(typename boost::function<RES_T(ARG_T)> f,
			typename boost::function<GRAD_T(ARG_T)> df) :
			FunctionWrapper<RES_T, ARG_T>(f), _df(df) {
	}

	//! Destructor
	virtual ~Function1DiffWrapper() {
	}

	/**
	 * @copydoc Function::df
	 *
	 * Redirects gradient evaluation to the wrapped callback.
	 */
	virtual GRAD_T df(ARG_T q) {
		return _df(q);
	}

	//! Get the wrapped callback.
	typename boost::function<RES_T(ARG_T)> getGradientCallback() {
		return _df;
	}

private:
	typename boost::function<GRAD_T(ARG_T)> _df;
};

} /* namespace math */
} /* namespace rw */

#endif /* SRC_RW_MATH_FUNCTIONWRAPPER_HPP_ */
