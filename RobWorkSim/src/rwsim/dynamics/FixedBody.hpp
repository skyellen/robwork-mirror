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

#ifndef RWSIM_DYNAMICS_FIXEDBODY_HPP_
#define RWSIM_DYNAMICS_FIXEDBODY_HPP_

//! @file FixedBody.hpp

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

	/**
	 * @brief a body with a fixed position, zero velocity and zero force.
	 *
	 * This body type is not allowed to move during simulation.
	 */
    class FixedBody : public Body
    {
    public:
        typedef rw::common::Ptr<FixedBody> Ptr;

        /**
         * @brief constructor
         * @param info [in] body information
         * @param bodyframe [in] the body frame
         * @param geoms [in] geometry
         */
    	FixedBody(const BodyInfo& info, rw::models::Object::Ptr obj):
    	    Body(info, obj)
    	{

    	}

    	//! @brief destructor
    	virtual ~FixedBody(){}

    public: // inheritet from Body interface

    	//! @copydoc Body::getPointVelW
        virtual rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const {
        	return rw::math::Vector3D<>(0,0,0);
        };

        rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const{
            return rw::math::VelocityScrew6D<>(0,0,0,0,0,0);
        }

        //! @copydoc Body::reset
    	void reset(rw::kinematics::State &state){}

    	//! @copydoc Body::calcEnergy
        double calcEnergy(const rw::kinematics::State& state,
        		const rw::math::Vector3D<>& gravity = rw::math::Vector3D<>::zero(),
				const rw::math::Vector3D<>& potZero = rw::math::Vector3D<>::zero()) const { return 0; };

        //! @copydoc Body::setForce
        void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){};

        //! @copydoc Body::getForce
        rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const{
            return rw::math::Vector3D<>(0,0,0);
        }

        //! @copydoc Body::addForce
        void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state){};

        //! @copydoc Body::setTorque
        void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){};

        //! @copydoc Body::addTorque
        void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state) {};

        //! @copydoc Body::getTorque
        rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
            return rw::math::Vector3D<>(0,0,0);
        };


    };
    //! @}
}
}

#endif /*FIXEDBODY_HPP_*/
