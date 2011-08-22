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

#ifndef RW_DYNAMICS_ACCESSOR_HPP_
#define RW_DYNAMICS_ACCESSOR_HPP_

//! @file Accessor.hpp

#include <rw/kinematics/FrameProperty.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/models/RigidBodyInfo.hpp>

namespace rwsim {
namespace dynamics{
	//! @addtogroup dynamics
	//! @{

	/**
	 * @brief a set of accessor functions for accessing frame properties regarding
	 * Dynamic stuff.
	 */
	class Accessor
	{
	public:
		/** @brief Accessor for RigidBodyInfo
		 *
		 * This is used to get RigidBodyInfo
		 */
		static const rw::kinematics::FrameProperty<rw::models::RigidBodyInfo>& RigidBodyInfo();

	};

	//! @}
}
}
#endif /*ACCESSOR_HPP_*/
