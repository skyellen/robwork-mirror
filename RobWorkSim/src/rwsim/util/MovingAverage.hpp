/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSIM_UTIL_MOVINGAVERAGE_HPP_
#define RWSIM_UTIL_MOVINGAVERAGE_HPP_

#include <vector>

namespace rwsim {
namespace util {

	/**
	 * @brief
	 */
	class MovingAverage {
	public:
		MovingAverage(int len):
			_len(len),_invLen(1.0/len),_cb(_len,0.0),_sum(0.0),_idx(0)
		{}

		void addSample(double sample){
			_sum -= _cb[_idx];
			_sum += sample;
			_cb[_idx] = sample;
			_idx++;
			if(_idx == _len )
				_idx = 0;
		}

		double getAverage(){
			return _sum*_invLen;
		}

	private:
		const int _len;
		double _invLen;
		std::vector<double> _cb;
		double _sum;
		int _idx;

	};
}
}

#endif /* MOVINGAVERAGE_HPP_ */
