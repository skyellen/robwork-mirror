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

#ifndef RWSIM_UTIL_GRASPPOLICYFACTORY_HPP_
#define RWSIM_UTIL_GRASPPOLICYFACTORY_HPP_

#include "GraspPolicy.hpp"

namespace rwsim { namespace dynamics { class DynamicDevice; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsim {
namespace util {
	class GraspPolicyFactory {
	public:

		static std::vector<std::string> getAvailablePolicies();

		static GraspPolicy::Ptr makePolicy(
				const std::string& id,
				dynamics::DynamicWorkCell* dwc,
				rwsim::dynamics::DynamicDevice* dev);

	};
}
}
#endif /* GRASPPOLICYFACTORY_HPP_ */
