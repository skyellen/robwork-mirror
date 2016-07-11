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

#ifndef RWSIM_UTIL_COLLISIONFREESAMPLER_HPP_
#define RWSIM_UTIL_COLLISIONFREESAMPLER_HPP_

#include "StateSampler.hpp"

namespace rw { namespace proximity { class CollisionDetector; } }

namespace rwsim {
namespace util {
	//! @addtogroup rwsim_util
	//! @{

	/**
	 * @brief samples another state sampler until it returns a collision free
	 * state.
	 */
	class CollisionFreeSampler: public StateSampler
	{
	public:

		/**
		 * @brief constructor
		 * @param sampler [in] the sampler that is to be wrapped
		 * @param detector [in] the collision detector
		 * @param n [in] max nr of tries pr sample request
		 */
		CollisionFreeSampler(StateSampler::Ptr sampler, rw::common::Ptr<rw::proximity::CollisionDetector> detector, int n=-1);

		/**
		 * @brief destructor
		 */
		virtual ~CollisionFreeSampler();

		//! @copydoc StateSampler::sample
		bool sample(rw::kinematics::State& state);

		//! @copydoc StateSampler::empty
		bool empty() const{ return _sampler->empty(); };

	private:
		StateSampler::Ptr _sampler;
		rw::common::Ptr<rw::proximity::CollisionDetector> _detector;
		int _n;
	};
	//! @}
}
}
#endif /* FINITESTATESAMPLER_HPP_ */
