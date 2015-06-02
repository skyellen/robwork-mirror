/*
 * Types.hpp
 *
 *  Created on: Feb 9, 2015
 *      Author: dagothar
 */

#ifndef SRC_RWLIBS_OPTIMIZATION_TYPES_HPP_
#define SRC_RWLIBS_OPTIMIZATION_TYPES_HPP_

#include <rw/math/Q.hpp>
#include <rw/math/Function.hpp>

namespace rwlibs {
namespace optimization {

/** @addtogroup optimization */
/*@{*/

//! Arithmetic type for optimized functions results.
typedef double ResultType;

//! Vector type for n-dimensional functions arguments.
typedef rw::math::Q VectorType;

//! Gradient type for n-dimensional functions.
typedef rw::math::Q GradientType;

//! n-dimensional function type: f(VectorType) -> double.
typedef rw::math::Function<double, VectorType> FunctionType;

//! n-dimensional function type: f(VectorType) -> double with derivative.
typedef rw::math::Function1Diff<double, VectorType, GradientType> Function1DiffType;

/*@}*/

}
}

#endif /* SRC_RWLIBS_OPTIMIZATION_TYPES_HPP_ */
