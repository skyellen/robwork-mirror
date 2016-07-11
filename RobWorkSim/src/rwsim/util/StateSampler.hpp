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

#ifndef RWSIM_UTIL_STATESAMPLER_HPP_
#define RWSIM_UTIL_STATESAMPLER_HPP_

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace util {

	/**
	 * @brief interface for generating states.
	 */
	class StateSampler {
		public:

	    typedef rw::common::Ptr<StateSampler> Ptr;

	    /**
			   @brief Sample a state.

			   If sampling fails, the sampler may return the empty configuration. If
			   empty() is true then the sampler has no more configurations.
			   Otherwise sample() may (or may not) succeed if called a second time.
			*/
			virtual bool sample(rw::kinematics::State& state) = 0;

			/**
			   @brief True if the sampler is known to contain no more
			   configurations.
			*/
			virtual bool empty() const = 0;
	};
}
}

#endif /* STATESAMPLER_HPP_ */
