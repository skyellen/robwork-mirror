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
	//! @addtogroup dynamics
	//! @{

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
                   rw::kinematics::MovableFrame &j,
                   const std::vector<rw::geometry::Geometry::Ptr>& geoms,
                   rw::kinematics::State &state);

    	virtual ~KinematicBody();

    public: // functions that need to be implemented by specialized class

    	rw::kinematics::MovableFrame* getMovableFrame(){ return _base; };

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
        void reset(rw::kinematics::State &state);

        /**
         * @copydoc Body::calcEnergy
         */
        double calcEnergy(const rw::kinematics::State &state) {return 0;};


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

    public:
        /**
         * @brief returns the linear velocity described in parent frame
         */
        rw::math::Vector3D<> getLinVel(const rw::kinematics::State& state) const {
        	const double *q = this->getData(state);
            return rw::math::Vector3D<>(q[0],q[1],q[2]);
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const {
            const double *q = this->getData(state);
        	rw::math::Vector3D<> v(q[3],q[4],q[5]);
        	return v;
        }

        /**
         * @brief returns the linear velocity described in parent frame
         */
        rw::math::Vector3D<> getLinVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getLinVel(state);
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        rw::math::Vector3D<> getAngVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getAngVel(state);
        }

        /**
         * @brief sets the linear velocity described in parent frame
         */
        void setLinVel(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            double *q = this->getData(state);
            q[0] = vel[0];
            q[1] = vel[1];
            q[2] = vel[2];
        }

        void setLinVelW(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            setLinVel( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * vel, state);
        }

        /**
         * @brief sets the angular velocity described in parent frame
         */
        void setAngVel(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            double *q = this->getData(state);
            q[3] = vel[0];
            q[4] = vel[1];
            q[5] = vel[2];
        }

        void setAngVelW(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            setAngVel( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * vel, state);
        }


    private:

        rw::kinematics::MovableFrame *_base;
    };
    //! @}
}
}

#endif /*LINK_HPP_*/
