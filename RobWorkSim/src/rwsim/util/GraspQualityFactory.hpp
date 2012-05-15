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

#ifndef RWSIM_UTIL_GRASPQUALITYFACTORY_HPP_
#define RWSIM_UTIL_GRASPQUALITYFACTORY_HPP_

#include <rw/graspplanning/GraspQualityMeasure3D.hpp>

namespace rwsim {
namespace util {

	class GraspQualityFactory {
	public:

		/**
		 * @brief a list of available quality metrics.
		 */
		static std::vector<std::string> getAvailableQualityMeasures();

		/**
		 * @brief instantiate a quality measure with ID \b id.
		 * @param id [in] id of quality measure
		 */
		static GraspQualityMeasure3DPtr makeQualityMeasure(const std::string& id);

	};
}
}
#endif /* GRASPSTRATEGYFACTORY_HPP_ */
