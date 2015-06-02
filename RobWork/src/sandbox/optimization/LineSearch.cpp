/*
 * LineSearch.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "LineSearch.hpp"

namespace rwlibs {
namespace optimization {

LineSearch::LineSearch() :
		Optimizer(NULL) {
}

LineSearch::LineSearch(typename FunctionType::Ptr function,
		VectorType direction, double left, double right) :
		Optimizer(function), _direction(direction / direction.norm2()), _leftBracket(
				left), _rightBracket(right) {
}

} /* namespace optimization */
} /* namespace rwlibs */
