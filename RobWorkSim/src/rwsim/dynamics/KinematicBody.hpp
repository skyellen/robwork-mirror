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

#include <rw/kinematics/Frame.hpp>

#include <rw/kinematics/Kinematics.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Vector3D.hpp>

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
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
    	//! @brief Smart pointer type for a KinematicBody.
        typedef rw::common::Ptr<KinematicBody> Ptr;

        //! @copydoc Body::Body()
        KinematicBody(const BodyInfo& info, rw::models::Object::Ptr obj);

        //! @brief Destructor.
    	virtual ~KinematicBody();

    public: // functions that need to be implemented by specialized class

    	/**
    	 * @brief Get the body frame as a movable frme.
    	 * @return a pointer to a movable frame.
    	 */
    	rw::kinematics::MovableFrame* getMovableFrame(){ return _base; };

        /**
         * @copydoc Body::getPointVelW
         */
        //rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& wPp, const rw::kinematics::State& state) const;

        rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const;

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
        double calcEnergy(const rw::kinematics::State& state,
        		const rw::math::Vector3D<>& gravity = rw::math::Vector3D<>::zero(),
				const rw::math::Vector3D<>& potZero = rw::math::Vector3D<>::zero()) const { return 0; }


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
            return _kstate.get(state).linvel;
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const {
            return _kstate.get(state).angvel;
        }

        /**
         * @brief returns the linear velocity described in world frame
         */
        rw::math::Vector3D<> getLinVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getLinVel(state);
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        rw::math::Vector3D<> getAngVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getAngVel(state);
        }

        /**
         * @brief sets the linear velocity described in parent frame
         */
        void setLinVel(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            _kstate.get(state).linvel = vel;
        }

        /**
         * @brief Set the linear velocity described in world frame.
         * @param vel [in] the linear velocity.
         * @param state the state giving the current pose of the body.
         */
        void setLinVelW(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            setLinVel( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * vel, state);
        }

        /**
         * @brief sets the angular velocity described in parent frame
         */
        void setAngVel(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            _kstate.get(state).angvel = vel;
        }

        /**
         * @brief Set the angular velocity described in world frame.
         * @param vel [in] the angular velocity.
         * @param state the state giving the current pose of the body.
         */
        void setAngVelW(const rw::math::Vector3D<>& vel, rw::kinematics::State& state) {
            setAngVel( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * vel, state);
        }

    protected:
        //! @brief State data for a kinematic body.
        struct KinematicBodyState {
        	//! @brief The linear velocity of the body.
            rw::math::Vector3D<> linvel;
        	//! @brief The angular velocity of the body.
            rw::math::Vector3D<> angvel;
        };



    private:
        rw::kinematics::MovableFrame *_base;
        rw::kinematics::StatelessData<KinematicBodyState> _kstate;

    };
    //! @}
}
}

#endif /*LINK_HPP_*/
