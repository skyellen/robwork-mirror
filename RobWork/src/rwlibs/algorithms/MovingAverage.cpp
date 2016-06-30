/*
 * MovingAverage.cpp
 *
 *  Created on: 09-12-2008
 *      Author: jimali
 */

#include "MovingAverage.hpp"

#include <rw/common/macros.hpp>

MovingAverage::MovingAverage(std::size_t N):
	_len((int)N),_invLen(1.0),_cb(_len,0.0),_sum(0.0),_idx(0)
{
	RW_ASSERT(N!=0);
	_invLen = 1.0/N;
}
