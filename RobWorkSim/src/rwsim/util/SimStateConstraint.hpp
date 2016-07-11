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

#ifndef RWSIM_UTIL_SIMSTATECONSTRAINT_HPP_
#define RWSIM_UTIL_SIMSTATECONSTRAINT_HPP_

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }

namespace rwsim {
namespace util {

	class SimStateConstraint {
	public:

	    typedef rw::common::Ptr<SimStateConstraint> Ptr;

	    virtual bool isSatisfied(const rw::kinematics::State &state, simulator::DynamicSimulator *sim) = 0;
	};
}
}
#endif /* SIMSTATECONSTRAINT_HPP_ */
