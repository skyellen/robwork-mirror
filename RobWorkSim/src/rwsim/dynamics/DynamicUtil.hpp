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

#ifndef RWSIM_DYNAMICS_DYNAMICUTIL_HPP_
#define RWSIM_DYNAMICS_DYNAMICUTIL_HPP_

//! @file DynamicUtil.hpp

#include <vector>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Transform3D.hpp>

#include <rw/geometry/Geometry.hpp>

#include "DynamicWorkCell.hpp"
#include "RigidBody.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup drawable @{

	/**
	 * @brief Utility functions for calculating dynamic properties
	 */
	class DynamicUtil
	{
	public:

		/**
		 * @brief util function that locates all frames that is staticly connected to f
		 * and that has geometry information.
		 */
		static std::vector<rw::kinematics::Frame*>
			getAnchoredFrames(rw::kinematics::Frame &f,
							  const rw::kinematics::State &state);

		/**
		 * @brief util function that locates all frames in the sub tree of parent
		 * that is staticly connected and that has geometry information.
		 */
		static std::vector<rw::kinematics::Frame*>
			getAnchoredChildFrames(rw::kinematics::Frame *parent, const rw::kinematics::State &state);
		//static double getMaxVelocity(dynamics::DynamicWorkcell& dwc);

		/**
		 * @brief
		 * @param parent
		 * @param state
		 * @param exclude
		 * @return
		 */
		static std::vector<rw::kinematics::Frame*>
			getAnchoredChildFrames(rw::kinematics::Frame *parent,
								   const rw::kinematics::State &state,
								   const std::vector<rw::kinematics::Frame*>& exclude);

		/**
		 * @brief get rigid bodies from a dynamic workcell
		 * @param dwc
		 * @return all rigid bodies in a dynamic workcell
		 */
		static std::vector<RigidBody*> getRigidBodies(DynamicWorkCell& dwc);


		static bool isResting(DynamicWorkCell::Ptr dwc,
		                      const rw::kinematics::State& state,
		                      double max_linvel = 0.01,
		                      double max_angvel = 0.1,
		                      double max_jointvel = 0.002);

        static bool isResting(DynamicDevice::Ptr dev,
                              const rw::kinematics::State& state,
                              double max_jointvel = 0.02);

	};
	//! @}
}
}

#endif /*DYNAMICUTIL_HPP_*/
