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

#ifndef RWSIM_DYNAMICS_KINEMATICDEVICE_HPP_
#define RWSIM_DYNAMICS_KINEMATICDEVICE_HPP_

#include <rw/models/JointDevice.hpp>

#include "DynamicDevice.hpp"

namespace rw { namespace models { class Object; } }

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

	/**
	 * @brief a kinematic device is able to influence the simulated environment
	 * but the device is not influenced by any external force as is the RigidDevice.
	 *
	 * The kinematic device is velocity controlled and if the acceleration limits permit
	 * then the velocity of the kinematic device will be equal to the target velocity in
	 * the next step of the simulator.
	 *
	 * This class is especially usefull for animating robot devices in a
	 * simulated environment.
	 */
	class KinematicDevice: public DynamicDevice {
	public:
		KinematicDevice(dynamics::Body::Ptr base,
	                    const std::vector<std::pair<BodyInfo,rw::common::Ptr<rw::models::Object> > >& objects,
						rw::models::JointDevice::Ptr dev);

		//! @brief destructor
		virtual ~KinematicDevice();

		//! @copydoc DynamicDevice::getJointVelocities
		rw::math::Q getJointVelocities(const rw::kinematics::State& state){
		    return rw::math::Q(_velocity.getN(), &_velocity.get(state) );
		};

        //! @copydoc DynamicDevice::setJointVelocities
		void setJointVelocities(const rw::math::Q &vel, rw::kinematics::State& state){
		    double *vals = &_velocity.get(state);
		    for(int i=0; i< std::min(_velocity.getN(), (int)vel.size()); i++ ){
		        vals[i] = vel[i];
		    }
		};

		/**
		 * @brief get the kinematic bodies that this KinematicDevice controls. The
		 * bodies are ordered such that device joint \b i maps to kinematic body  \b i
		 * @return all bodies that the device controls.
		 */
		const std::vector<Body::Ptr>& getLinks(){ return _links; }

		// parameters for velocity profile
		void setMaxAcc(const rw::math::Q& acc);
		rw::math::Q getMaxAcc();

		void setMaxVel(const rw::math::Q& vel);
		rw::math::Q getMaxVel();

		rw::models::JointDevice::Ptr getJointDevice(){ return _jdev; }
	private:
		std::vector<Body::Ptr> _links;
		rw::math::Q _maxVel, _maxAcc;
		//rw::math::Q _q, _velQ;
		rw::models::JointDevice::Ptr _jdev;
		rw::kinematics::StatelessData<double> _velocity;
	};
	//! @}
}
}

#endif /*KINEMATICDEVICE_HPP_*/
