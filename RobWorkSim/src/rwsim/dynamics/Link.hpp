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

#ifndef RWSIM_DYNAMICS_LINK_HPP_
#define RWSIM_DYNAMICS_LINK_HPP_

//! @file Link.hpp

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/Device.hpp>

#include "Body.hpp"

namespace rwsim {
namespace dynamics {

    class DynamicDevice;

	//! @addtogroup rwsim_dynamics
	//! @{
	/**
	 * @brief The Link is a body that is part of a dynamic device where joints are used to
	 * constrain the movement of links.
	 *
	 * As such the Link has no state variables and relies purely on the state of the
	 * dynamic device to provide its velocity and acceleration. The link class is therefore
	 * merilly a wrapper/convienience interface for accessing methods on the robot links.
	 */
    class Link : public Body
    {
    public:
        Link( const BodyInfo& info, rw::models::Object::Ptr obj, DynamicDevice *ddev, size_t id);

    	virtual ~Link();

    public: // functions that need to be implemented by specialized class

    	//! @copydoc Body::getVelocity
    	virtual rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const = 0;

    	 virtual void reset(rw::kinematics::State &state);

     	//! @copydoc Body::calcEnergy
         virtual double calcEnergy(const rw::kinematics::State& state,
         		const rw::math::Vector3D<>& gravity = rw::math::Vector3D<>::zero(),
 				const rw::math::Vector3D<>& potZero = rw::math::Vector3D<>::zero()) const;

    	 virtual void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state);

    	 virtual rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const;

    	 virtual void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state);

    	 virtual void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state);

    	 virtual void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state);

    	 virtual rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const;

    	 DynamicDevice* getDynamicDevice(){ return _ddev; }

    	 size_t getID(){ return _id; }
    private:
    	 rw::models::Object::Ptr _obj;
    	DynamicDevice *_ddev;
    	size_t _id;
    };
    //! @}
}
}

#endif /*LINK_HPP_*/
