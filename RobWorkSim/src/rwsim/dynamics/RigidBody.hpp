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

#ifndef RWSIM_DYNAMICS_RIGIDBODY_HPP_
#define RWSIM_DYNAMICS_RIGIDBODY_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/InertiaMatrix.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/kinematics/Kinematics.hpp>

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{
    /**
     * @brief A body is an abstraction over some physical object in the scene.
     * The body class is an abstract class that allows interaction by adding gravity,
     * forces and impulses. The integration scheme used to update the velocity and position
     * of the body is defined by the class that inherit the body interface.
     *
     *
     * The rigid body hold 12 state variables in the StateStructure beside the 6 allocated by
     * MovableFrame (for position). the first 6 state variables are for velocity(linear,angular), and the last six
     * are for force/torque accumulation (force,torque)
     */
    class RigidBody : public Body
    {
    public:
        typedef rw::common::Ptr<RigidBody> Ptr;
        /**
         * @brief constructor
         * @param info [in] body description
         * @param frame [in] body reference frame
         * @param geom [in] geometry
         */
        RigidBody(const BodyInfo& info, rw::models::Object::Ptr obj);

        //RigidBody(std::vector<std::pair<BodyInfo, rw::models::Object::Ptr> > infos );

    	//! @brief destructor
        virtual ~RigidBody(){};

    public: // functions that need to be implemented by specialized class

        /**
         * @copydoc Body::getPointVelW
         */
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const;

        rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const;
        /**
         * @copydoc Body::reset
         */
        void reset(rw::kinematics::State &state);

        /**
         * @copydoc Body::calcEnergy
         * @note RigidBody energy = 1/2 * mv^2 + 1/2 * Iw^2 + mgz
         */
        double calcEnergy(const rw::kinematics::State& state);

        //! @copydoc Body::setForce
        void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
            _rstate.get(state).force = f;
        }

        //! @copydoc Body::addForce
        void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state){
            _rstate.get(state).force += force;
        }

        //! @copydoc Body::getForce
        rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const {
            return _rstate.get(state).force;
        }

        //! @copydoc Body::setTorque
        void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            _rstate.get(state).torque = t;
        }

        //! @copydoc Body::addTorque
        void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            _rstate.get(state).torque += t;
        }

        //! @copydoc Body::getTorque
        rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
            return _rstate.get(state).torque;
        }

    public:

        /**
         *
         */
        //rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc);

        /**
         * @brief return the parent frame
         */
    	rw::kinematics::Frame* getParent(const rw::kinematics::State& state) const {
    		return _mframe->getParent(state);
    	};

    	/**
    	 * @brief body type
    	 */
    	int getBodyType() const {
    	    return _bodyType;
    	};


        /**
         * @brief returns the transform from parent to body
         */
        rw::math::Transform3D<> getPTBody(const rw::kinematics::State& state) const {
            return _mframe->getTransform(state);
        }

        /**
         * @brief set the transform of the body. The transform is described relative to parent.
         */
        void setPTBody(const rw::math::Transform3D<>& pTb, rw::kinematics::State& state){
            _mframe->setTransform( pTb , state );
        }

        /**
         * @brief returns the transform from world to body
         */
        rw::math::Transform3D<> getWTBody(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(_mframe, state);
        }

        rw::math::Transform3D<> getWTParent(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParent(state), state);
        }

        /**
         * @brief return the linear velocity described in parent frame
         */
        rw::math::Vector3D<> getLinVel(const rw::kinematics::State& state) const {
            return _rstate.get(state).linvel;
        	//const double *q = this->getData(state);
            //return rw::math::Vector3D<>(q[0],q[1],q[2]);
        }

        /**
         * @brief return the linear velocity described in world frame
         */
        rw::math::Vector3D<> getLinVelW(const rw::kinematics::State& state) const {
        	return getWTParent(state).R() * getLinVel(state);
        }

        /**
         * @brief set the linear velocity.
         */
        void setLinVel(const rw::math::Vector3D<> &lvel, rw::kinematics::State& state){
            _rstate.get(state).linvel = lvel;
        }

        void setLinVelW(const rw::math::Vector3D<> &lvel, rw::kinematics::State& state){
            setLinVel( inverse(getWTParent(state).R()) * lvel, state);
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const {
            return _rstate.get(state).angvel;
            //const double *q = this->getData(state);
        	//return rw::math::Vector3D<>(q[3],q[4],q[5]);
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        virtual rw::math::Vector3D<> getAngVelW(const rw::kinematics::State& state) const {
            return getWTParent(state).R() * getAngVel(state);
        }

        /**
         * @brief set the angular velocity of this rigid body.
         */
        virtual void setAngVel(const rw::math::Vector3D<> &avel, rw::kinematics::State& state);

        void setAngVelW(const rw::math::Vector3D<> &avel, rw::kinematics::State& state){
            setAngVel(inverse(getWTParent(state).R())*avel, state);
        }

        /**
         * @brief calculates the relative velocity in parent frame of a point p on the body
         * described in parent frame.
         */
        rw::math::Vector3D<> getPointVel(const rw::math::Vector3D<>& p, const rw::kinematics::State& state){
            return getLinVel(state) + cross(getAngVel(state), p);
        }

        /**
         * @brief returns the mass of this body
         */
        inline double getMass() const {
            return getInfo().mass;
        }

        /**
         * @brief returns the inverse of the mass of this body
         */
        inline double getMassInv() const {
            return _massInv;
        }

        /**
         * @brief returns the body inertia matrix
         */
        const rw::math::InertiaMatrix<>& getBodyInertia() const {
            return _Ibody;
        };

        /**
         * @brief return the inverse of the body inertia matrix
         */
        const rw::math::InertiaMatrix<>& getBodyInertiaInv() const {
            return _IbodyInv;
        };

        /**
         * @brief Get the principal inertia of this body. The inertia is described
         * around the center of mass for the principal axes.
         * @return the principal inertia and the principal axes as a rotation matrix.
         */
        const std::pair<rw::math::Rotation3D<>, rw::math::Vector3D<> >& getBodyPrincipalInertia() const {
        	return _IbodyPrincipal;
        };

        /**
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        rw::math::InertiaMatrix<> calcInertiaTensorInv(const rw::kinematics::State& state) const;

        /**
         * @brief returns the inverse of the inertia tensor described in
         * world frame.
         */
        //rw::math::InertiaMatrix<> calcInertiaTensorInvW(const rw::kinematics::State& state) const;

        /**
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        rw::math::InertiaMatrix<> calcInertiaTensor(const rw::kinematics::State& state) const;


        rw::kinematics::MovableFrame* getMovableFrame(){
        	return _mframe;
        }

        /**
         * @brief calculates the effective mass of this rigid body seen from
         * the contact point \b wPc. \b wPc is described relative to parent frame
         */
        rw::math::InertiaMatrix<> calcEffectiveMass(const rw::math::Vector3D<>& wPc, const rw::kinematics::State& state) const;

        /**
         * @brief calculates the effective mass of this rigid body seen from
         * the contact point \b wPc. \b wPc is described relative to world
         */
        rw::math::InertiaMatrix<> calcEffectiveMassW(const rw::math::Vector3D<>& wPc, const rw::kinematics::State& state) const;

        //rw::math::InertiaMatrix<> calcEffectiveInertia(const rw::kinematics::State& state) const;

        //rw::math::InertiaMatrix<> calcEffectiveInertiaInv(const rw::kinematics::State& state) const;


    protected:
        // const variables
        const double _mass, _massInv;

        struct RigidBodyState {
            rw::math::Vector3D<> linvel;
            rw::math::Vector3D<> angvel;
            rw::math::Vector3D<> force;
            rw::math::Vector3D<> torque;
        };




        rw::kinematics::MovableFrame *_mframe;
        //rw::kinematics::Frame *_parent;

        // TODO: body type
        int _bodyType;

        // inertia tensors
        const rw::math::InertiaMatrix<> _Ibody;
        const rw::math::InertiaMatrix<> _IbodyInv;
        const std::pair<rw::math::Rotation3D<>, rw::math::Vector3D<> > _IbodyPrincipal;

        // state variables
        rw::math::InertiaMatrix<> _ITensorInv,_ITensor; // inverse inertia tensor in parent frame

        rw::kinematics::StatelessData<RigidBodyState> _rstate;
    };
    //! @}
} // namespace dynamics
}

#endif /*DYNAMICS_RIGIDBODY_HPP_*/
