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

#ifndef RWSIM_UTIL_PRESHAPESAMPLER_HPP_
#define RWSIM_UTIL_PRESHAPESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/kinematics/State.hpp>

namespace rw { namespace pathplanning { class QSampler; } }
namespace rw { namespace models { class Device; } }

namespace rwsim {
namespace util {


	/**
	 * @brief
	 *
	 * This StateSampler will never become empty
	 *
	 */
	class PreshapeSampler: public StateSampler
	{
	public:

		/**
		 * @brief create a preshape sampler based on a QSampler
		 * @param dev [in] the device for which configurations are sampled
		 * @param qsampler [in] the configuration sampler
		 * @param initState [in] the initial state
		 */
		PreshapeSampler(rw::models::Device* dev,
						rw::common::Ptr<rw::pathplanning::QSampler> qsampler,
						rw::kinematics::State& initState);

		/**
		 * @brief destructor
		 */
		virtual ~PreshapeSampler();



		//! @copydoc StateSampler::sample
		bool sample(rw::kinematics::State& state);

		//! @copydoc StateSampler::sample
		bool empty() const{ return false; };

	private:
		rw::models::Device* _dev;
		rw::common::Ptr<rw::pathplanning::QSampler> _qsampler;
		rw::kinematics::State _initState;
	};
}
}

#endif /* FINITESTATESAMPLER_HPP_ */
