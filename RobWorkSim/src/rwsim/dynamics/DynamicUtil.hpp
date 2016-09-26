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

#include "RigidBody.hpp"

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace dynamics {
	class DynamicDevice;
	class DynamicWorkCell;

	//! @addtogroup drawable
	//! @{

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
		static std::vector<RigidBody::Ptr> getRigidBodies(DynamicWorkCell& dwc);

		/**
		 * @brief Check if the dynamic workcell has reached a steady state where objects are in rest.
		 * @param dwc [in] the dynamic workcell.
		 * @param state [in] the current state.
		 * @param max_linvel [in] (optional) the linear velocity threshold. Default is 0.02 m/s.
		 * @param max_angvel [in] (optional) the angular velocity threshold. Default is 0.1 rad/s.
		 * @param max_jointvel [in] (optional) the joint velocity threshold. Default is 0.05 rad/s.
		 * @return
		 */
		static bool isResting(rw::common::Ptr<DynamicWorkCell> dwc,
		                      const rw::kinematics::State& state,
		                      double max_linvel = 0.02,
		                      double max_angvel = 0.1,
		                      double max_jointvel = 0.05);

		/**
		 * @brief Check if a device has reached a steady state where it is in rest.
		 * @param dev [in] the device.
		 * @param state [in] the current state.
		 * @param max_linjointvel [in] (optional) the linear joint velocity threshold. Default is 0.02 m/s.
		 * @param max_jointvel [in] (optional) the angular joint velocity threshold. Default is 0.05 rad/s.
		 * @return
		 */
        static bool isResting(rw::common::Ptr<DynamicDevice> dev,
                              const rw::kinematics::State& state,
                              double max_linjointvel = 0.02,
                              double max_jointvel = 0.05);


        /**
         * @brief compute torques on a robot arm as a concequence of gravity, position, velocity and acceleration.
         * @param q [in] joint position
         * @param dq [in] joint velocity
         * @param ddq [in] joint acceleration
         * @param dev [in] device
         * @param gravity [in] gravity vector
         * @return
         */
        /*
        static rw::math::Q computeTorques(const rw::kinematics::State& defstate,const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& ddq,
        								  dynamics::RigidDevice::Ptr dev, const rw::math::Vector3D<>& gravity=rw::math::Vector3D<>(0,0,-9.82));
	*/
	};
	//! @}
}
}

#endif /*DYNAMICUTIL_HPP_*/
