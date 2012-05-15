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

#ifndef RWSIM_UTIL_GRASPSTRATEGY_HPP_
#define RWSIM_UTIL_GRASPSTRATEGY_HPP_

#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>
#include "StateSampler.hpp"

namespace rwsim {
namespace util {

    /**
     * @brief a GraspStrategy define how the initial configuration of a grasping
     * system is generated.
     *
     * Example:
     * A simple GraspStrategy is to randomly place the robot hand with its palm pointing toward
     * the object and randomly generate collision free finger configurations.
     *
     * Another example would be to load all start configurations from some file...
     */
	class GraspStrategy {
	public:

	    typedef rw::common::Ptr<GraspStrategy> Ptr;

		virtual StateSampler::Ptr getSampler() = 0;

		virtual std::string getIdentifier() = 0;

		virtual rw::common::PropertyMap& getSettings() = 0;

		virtual void applySettings() = 0;
	};

}
}
#endif /* GRASPSTRATEGY_HPP_ */
