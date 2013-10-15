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

#include <rw/kinematics/Stateless.hpp>

#include "Body.hpp"
#include "Link.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{
	/**
	 * @brief base class for dynamic devices that has dynamic state values
	 * such as velocity and acceleration.
	 */
	class DynamicDevice: public rw::kinematics::Stateless {

	public:
	    typedef rw::common::Ptr<DynamicDevice> Ptr;


		/**
		 * @brief destructor
		 */
		virtual ~DynamicDevice(){};

		/**
		 * @brief gets the position
		 */
		virtual rw::math::Q getQ(const rw::kinematics::State& state){
			return _dev->getQ(state);
		}

		virtual void setQ(const rw::math::Q &q, rw::kinematics::State& state){
            _dev->setQ(q, state);
        };

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

		dynamics::Body::Ptr getBase(){ return _base;};


		/**
		 * @brief get the current velocities of all joints
		 * @param state [in] the state
		 * @return velocites of all joints
		 */
		virtual rw::math::Q getJointVelocities(const rw::kinematics::State& state) = 0;
		virtual void setJointVelocities(const rw::math::Q &vel, rw::kinematics::State& state) = 0;


		/// deprecated
		virtual rw::math::Q getVelocity(const rw::kinematics::State& state){ return getJointVelocities(state); };
		virtual void setVelocity(const rw::math::Q &vel, rw::kinematics::State& state)
		    { setJointVelocities(vel, state); };

		virtual void setMotorVelocityTargets(const rw::math::Q& vel, rw::kinematics::State& state){ }

        virtual const std::vector<Body::Ptr>& getLinks() = 0;

        const std::string& getName() const { return _dev->getName(); };

	protected:

		DynamicDevice(dynamics::Body::Ptr base, rw::models::Device::Ptr dev):
			_dev(dev),
			_base(base)
		{}


		rw::models::Device::Ptr _dev;
		dynamics::Body::Ptr _base;
	private:
		DynamicDevice();

	};
	//! @}
}
}

#endif /*DYNAMICDEVICE_HPP_*/
