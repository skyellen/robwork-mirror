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

#ifndef RWSIM_DYNAMICS_KINEMATICBODY_HPP_
#define RWSIM_DYNAMICS_KINEMATICBODY_HPP_

#include <rw/models/Joint.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>

#include <rw/kinematics/Kinematics.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics @{

	/**
	 * @brief a kinematic body is a body that effects the motion of other objects but it is not
	 * directly affected itself.
	 *
	 * The user can make the kinematic object move using its velocity interface. Typically one
	 * would add a controller to do velocity updates of the kinematic body.
	 *
	 * The kinematic body stores a 6 dof velocity (angular, linear) in the state vector.
	 */
    class KinematicBody : public Body
    {
    public:
        KinematicBody(
                   const BodyInfo& info,
                   rw::kinematics::Frame &j,
                   const std::vector<rw::geometry::GeometryPtr>& geoms,
                   rw::kinematics::State &state);

    	virtual ~KinematicBody();

    public: // functions that need to be implemented by specialized class
        /**
         * @copydoc Body::saveState
         */
        virtual void saveState(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::rollBack
         */
        virtual void rollBack(rw::kinematics::State& state);

        /**
         * @copydoc Body::getPointVelW
         */
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& wPp, const rw::kinematics::State& state) const;

        /**
         * @copydoc Body::getEffectiveMassW
         */
        rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc);

        /**
         * @copydoc Body::resetState
         */
        void resetState(rw::kinematics::State &state);

        /**
         * @copydoc Body::calcEnergy
         */
        double calcEnergy(const rw::kinematics::State &state) {return 0;};

        /**
         * @brief returns the linear velocity described in parent frame
         */
        rw::math::Vector3D<> getLinVel(const rw::kinematics::State& state) const {
        	const double *q = this->getQ(state);
            return rw::math::Vector3D<>(q[0],q[1],q[2]);
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const {
            const double *q = this->getQ(state);
        	rw::math::Vector3D<> v(q[3],q[4],q[5]);
        	return v;
        }


    private:

        rw::kinematics::Frame *_base;
    };
    //! @}
}
}

#endif /*LINK_HPP_*/
