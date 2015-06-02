/*
 * StopCondition.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: dagothar
 */

#include "StopCondition.hpp"

//#include <iostream>

namespace rwlibs {
namespace optimization {

StopCondition::StopCondition(unsigned long maxSteps, double minTolerance) :
		_maxSteps(maxSteps), _minTolerance(minTolerance) {

}

//! Destructor.
StopCondition::~StopCondition() {
}

/**
 * @brief Is condition met?
 *
 * Checks optimizer state and returns \b true when consition is met.
 */
bool StopCondition::check(Optimizer::Ptr optimizer) const {
	//std::cout << optimizer->getIterationCount() << " " <<  optimizer->getCurrentError() << std::endl;

	if (optimizer->getStepCount() > _maxSteps
			|| optimizer->getCurrentError() < _minTolerance) {
		return true;
	} else {
		return false;
	}
}

} /* namespace optimization */
} /* namespace rwlibs */
