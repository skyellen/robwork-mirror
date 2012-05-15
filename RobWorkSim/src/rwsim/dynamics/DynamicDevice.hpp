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

#ifndef RWSIM_DYNAMICS_DYNAMICDEVICE_HPP_
#define RWSIM_DYNAMICS_DYNAMICDEVICE_HPP_

//! @file DynamicDevice.hpp

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include "Body.hpp"
#include "Link.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics
	//! @{
	/**
	 * @brief base class for dynamic devices that has dynamic state values
	 * such as velocity and acceleration.
	 */
	class DynamicDevice {

	public:
	    typedef rw::common::Ptr<DynamicDevice> Ptr;


		/**
		 * @brief destructor
		 */
		virtual ~DynamicDevice(){};

		/**
		 * @brief gets the position
		 */
		rw::math::Q getQ(const rw::kinematics::State& state){
			return _dev->getQ(state);
		}

		/**
		 * @brief gets the kinematic model of the DynamicDevice.
		 */
		rw::models::Device& getModel(){
			return *_dev;
		}

        rw::models::Device::Ptr getKinematicModel(){
            return _dev;
        }

		// Joint acceleration
		//void setQdd(const rw::kinematics::Q& qdd, const rw::kinematics::State& state);
		//rw::math::Q getQdd(const rw::kinematics::State& state);

		dynamics::Body* getBase(){ return _base;};

		virtual rw::math::Q getVelocity(const rw::kinematics::State& state) = 0;

		virtual void setVelocity(const rw::math::Q &vel, const rw::kinematics::State& state) = 0;

		/**
		 * @brief add force or torque (depending on joint type) to the joints of this
		 * device.
		 * @param forceTorque [in]
		 * @param state [in]
		 */
		virtual void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state) = 0;

		virtual void setForceLimit(const rw::math::Q& force){}

        virtual const std::vector<Body*>& getLinks() = 0;

	protected:

		DynamicDevice(dynamics::Body* base, rw::models::Device::Ptr dev):
			_dev(dev),
			_base(base)
		{}


		rw::models::Device::Ptr _dev;
		dynamics::Body* _base;
	private:
		DynamicDevice();

	};
	//! @}
}
}

#endif /*DYNAMICDEVICE_HPP_*/
