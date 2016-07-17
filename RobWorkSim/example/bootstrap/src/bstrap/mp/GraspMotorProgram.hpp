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

#ifndef GRASPMOTORPROGRAM_HPP_
#define GRASPMOTORPROGRAM_HPP_

#include <bstrap/core/MotorProgram.hpp>

#include <rw/math/Transform3D.hpp>

namespace rwsim { namespace control { class PoseController; } }

class BrainState;

class GraspMotorProgram: public MotorProgram {
public:

	GraspMotorProgram(const std::string& name,
			rw::common::Ptr<rwsim::control::PoseController> graspController);
	void setParameters(rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate);
	void executionloop();
	void update(const BrainState& state);
private:
	std::string _objName;
    rw::math::Transform3D<> _target;
    rw::common::Ptr<rwsim::control::PoseController> _graspController;

};


#endif
