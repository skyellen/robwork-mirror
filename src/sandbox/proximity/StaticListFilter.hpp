/*
 * StaticListFilter.hpp
 *
 *  Created on: Apr 23, 2009
 *      Author: jimali
 */

#ifndef STATICLISTFILTER_HPP_
#define STATICLISTFILTER_HPP_

#include "BroadPhaseDetector.hpp"

class StaticListFilter: BroadPhaseDetector {
public:

	StaticListFilter();

	StaticListFilter(const CollisionSetup& setup);

	void addInclude(const FramePair& framepair);

	void addExclude(const FramePair& framepair);

};

#endif /* STATICLISTFILTER_HPP_ */
