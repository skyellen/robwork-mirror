/*
 * PowellOptimizer.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: dagothar
 */

#include "PowellOptimizer.hpp"

#include <boost/foreach.hpp>

namespace rwlibs {
namespace optimization {

bool directionComp(const DirectionSetOptimizer::Direction& dir1,
		const DirectionSetOptimizer::Direction& dir2) {
	return (dir1.second < dir2.second);
}

PowellOptimizer::PowellOptimizer(typename FunctionType::Ptr function,
		double stepSize, StopCondition::Ptr stopCondition,
		LineSearch::Ptr strategy) :
		DirectionSetOptimizer(function, stepSize, stopCondition, strategy), _dirIdx(
				0) {

}

PowellOptimizer::~PowellOptimizer() {
}

void PowellOptimizer::newOptimization(const VectorType& initialGuess,
		ResultType& initialValue, double& initialError) {
	/*
	 * Build directions set from unit vectors.
	 */

	const std::size_t dims = initialGuess.size();

	DirectionSet& directionSet = getDirectionSet();
	directionSet.clear();

	for (std::size_t i = 0; i < dims; ++i) {
		VectorType baseDir(dims, 0.0);
		baseDir(i) = 1.0;

		directionSet.push_back(std::make_pair(baseDir, 0.0));
	}

	typename FunctionType::Ptr f = getFunction();
	initialValue = f->f(initialGuess);

	_dirIdx = 0;
}

void PowellOptimizer::step(VectorType& currentGuess, ResultType& currentValue,
		double& currentError) {
	/*
	 * Perform line search along the current dimension.
	 */
	LineSearch::Ptr lineSearch = getLineSearchStrategy();
	StopCondition::Ptr stopCondition = getLineSearchStopCondition();

	DirectionSet& directionSet = getDirectionSet();
	lineSearch->setDirection(directionSet[_dirIdx].first);

	VectorType prev = currentGuess; // guess before this step
	currentGuess = lineSearch->optimize(currentGuess, *stopCondition);
	currentValue = lineSearch->getCurrentValue();

	/*
	 * The quality of current directions is updated with:
	 * the distance the guess moved along it
	 */
	directionSet[_dirIdx].second = (currentGuess - prev).norm2();

	// update direction index
	++_dirIdx;

	if (_dirIdx >= directionSet.size()) {
		// calculate error
		ResultType dValue = currentValue - _previousValue;
		double distance = (currentGuess - _previousGuess).norm2();
		currentError = fabs(dValue / distance);

		_previousGuess = currentGuess;
		_previousValue = currentValue;

		// sort directions in ascending order
		std::sort(directionSet.begin(), directionSet.end(), directionComp);

		// create new direction
		Direction newDir(VectorType(currentGuess.size(), 0.0), 0.0);
		//double totalQuality = 0.0;
		BOOST_FOREACH(Direction & dir, directionSet) {
			//totalQuality += dir.second;
			newDir.first += dir.second * dir.first;
		}
		newDir.first /= newDir.first.norm2(); //totalQuality;
		directionSet.pop_back();
		directionSet.push_back(newDir);

		// return to the beginning of directions set
		_dirIdx = 0;
	}
}

} /* namespace optimization */
} /* namespace rwlibs */
