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
	//! @addtogroup dynamics @{

	/**
	 * @brief a body with a fixed position, zero velocity and zero force.
	 *
	 * This body type is not allowed to move during simulation.
	 */
    class FixedBody : public Body
    {
    public:
    	FixedBody(
    	    const BodyInfo& info,
    	    rw::kinematics::Frame *bodyframe,
            const std::vector<rw::geometry::GeometryPtr>& geoms):
    	    Body(0, info, bodyframe, geoms)
    	{

    	}

    	virtual ~FixedBody(){}

    public: // inheritet from Body interface

        virtual void saveState(double h, rw::kinematics::State& state){};

        virtual void rollBack(rw::kinematics::State& state){};

        virtual rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const{
        	return rw::math::Vector3D<>(0,0,0);
        };

    	 void resetState(rw::kinematics::State &state){}

    	 double calcEnergy(const rw::kinematics::State &state) {return 0;};

    };
    //! @}
}
}

#endif /*FIXEDBODY_HPP_*/
