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

#ifndef RWSIM_UTIL_GRASPPOLICY_HPP_
#define RWSIM_UTIL_GRASPPOLICY_HPP_

#include <rw/common/PropertyMap.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim {
namespace util {

/**
 * @brief a grasp policy defines how a grasp is executed from
 * some initial configuration.
 *
 * Typically this is some control
 * scheme that will close the fingers of a device into a grasp
 * of an object.
 *
 */
class GraspPolicy {
public:

	virtual void reset(const rw::kinematics::State& state) = 0;

	virtual rwlibs::simulation::SimulatedController* getController() = 0;

	virtual std::string getIdentifier() = 0;

	virtual rw::common::PropertyMap getSettings() = 0;

	virtual void applySettings() = 0;

};

typedef rw::common::Ptr<GraspPolicy> GraspPolicyPtr;

}
}


#endif /* GRASPPOLICY_HPP_ */
