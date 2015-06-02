/*
 * FunctionFactory.hpp
 *
 *  Created on: Feb 7, 2015
 *      Author: dagothar
 */

#ifndef SRC_RW_MATH_FUNCTIONFACTORY_HPP_
#define SRC_RW_MATH_FUNCTIONFACTORY_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Function.hpp>
#include <rw/math/FunctionWrapper.hpp>

namespace rw {
namespace math {

/** @addtogroup math */
/*@{*/

class FunctionFactory {
protected:
	template<typename RES_T, typename ARG_T>
	struct FPtr {
		typedef RES_T (*Ptr)(ARG_T);
	};

public:
	/**
	 * @brief Creates function from a function pointer.
	 *
	 * @param f [in] function value callback
	 */
	template<typename RES_T = double, typename ARG_T = double>
	static typename rw::math::Function<RES_T, ARG_T>::Ptr makeFunction(
			RES_T (*fptr)(ARG_T)) {

		return rw::common::ownedPtr(
				new rw::math::FunctionWrapper<RES_T, ARG_T>(
						boost::function<RES_T(ARG_T)>(fptr)));
	}

	/**
	 * @brief Creates function from a boost function object.
	 *
	 * @param f [in] function value callback
	 */
	template<class RES_T = double, class ARG_T = double>
	static typename rw::math::Function<RES_T, ARG_T>::Ptr makeFunction(
			boost::function<RES_T(ARG_T)> f) {
		return rw::common::ownedPtr(
				new rw::math::FunctionWrapper<RES_T, ARG_T>(f));
	}

	/**
	 * @brief Creates a differentiable function from function pointers.
	 *
	 * @param f [in] function value callback
	 * @param df [in] function derivative callback
	 */
	template<class RES_T = double, class ARG_T = double, class GRAD_T = double>
	static typename rw::math::Function1Diff<RES_T, ARG_T, GRAD_T>::Ptr makeFunction(
			RES_T (*fptr)(ARG_T), GRAD_T (*dfptr)(ARG_T)) {
	return rw::common::ownedPtr(
			new rw::math::Function1DiffWrapper<RES_T, ARG_T, GRAD_T>(boost::function<RES_T(ARG_T)>(fptr),
					boost::function<GRAD_T(ARG_T)>(dfptr)));
		}

		/**
		 * @brief Creates a differentiable function from boost function objects.
		 *
		 * @param f [in] function value callback
		 * @param df [in] function derivative callback
		 */
		template<class RES_T = double, class ARG_T = double,
				class GRAD_T = double>
		static typename rw::math::Function1Diff<RES_T, ARG_T, GRAD_T>::Ptr makeFunction(
				boost::function<RES_T(ARG_T)> f,
				boost::function<GRAD_T(ARG_T)> df) {
			return rw::common::ownedPtr(
					new rw::math::Function1DiffWrapper<RES_T, ARG_T, GRAD_T>(f,
							df));
		}
	};

	/*@}*/

	} /* namespace math */
	} /* namespace rw */

#endif /* SRC_RW_MATH_FUNCTIONFACTORY_HPP_ */
