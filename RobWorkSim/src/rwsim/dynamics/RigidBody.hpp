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
	//! @addtogroup dynamics @{
    /**
     * @brief A body is an abstraction over some physical object in the scene.
     * The body class is an abstract class that allows interaction by adding gravity,
     * forces and impulses. The integration scheme used to update the velocity and position
     * of the body is defined by the class that inherit the body interface.
     */
    class RigidBody : public Body
    {
    public:
    	/**
    	 * @brief constructor
    	 */
    	RigidBody(
            const BodyInfo& info,
            rw::kinematics::MovableFrame* frame,
            const std::vector<rw::geometry::GeometryPtr>& geoms,
            rw::kinematics::State& state
            );

    	//! @brief destructor
        virtual ~RigidBody(){};

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
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const;

        /**
         * @copydoc Body::getEffectiveMassW
         */
        rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc);

        /**
         * @copydoc Body::resetState
         */
        void resetState(rw::kinematics::State &state);

        /**
         * @copydoc Body::reset
         */
        virtual void reset(){
           rw::math::Vector3D<> zeroVec = rw::math::Vector3D<>(0.0,0.0,0.0);
           _force = zeroVec;
           _torque = zeroVec;
        }


    public:
        /**
         * @brief return the parent frame
         */
    	rw::kinematics::Frame* getParent() const {
    		return _parent;
    	};

    	/**
    	 * @brief body type
    	 */
    	int getBodyType() const {
    	    return _bodyType;
    	};

        /**
         * @brief Sets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
            _force = f;
        }

        /**
         * @brief Sets the force described in world frame acting on
         * the center mass of this body.
         *
         * @note more efficient to ude setForce()
         */
        virtual void setForceW(const rw::math::Vector3D<>& f, rw::kinematics::State& state);

        /**
         * @brief Gets the force described in parent frame acting on
         * the center mass of this body.
         */

        virtual const rw::math::Vector3D<>& getForce(const rw::kinematics::State& state){
            return _force;
        }

        /**
         * @brief Gets the force described in world frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForceW(const rw::kinematics::State& state) const{
            return rw::kinematics::Kinematics::worldTframe(_parent, state).R() * _force;
        }

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force, const rw::kinematics::State& state){
            _force += force;
        }

        /**
         * @brief Adds a force described in world frame to the
         * center of mass of this body
         */
        virtual void addForceW(const rw::math::Vector3D<>& force, rw::kinematics::State& state);

        /**
         * @brief Adds a force described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        virtual void addForceToPos(const rw::math::Vector3D<>& force,
                                   const rw::math::Vector3D<>& pos,
                                   rw::kinematics::State& state){
            // calculate the center force contribution
            _force += force;

            // calculate the torque contribution
            _torque += cross( pos, force );
        }

        /**
         * @brief Adds a force described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        virtual void addForceWToPosW(const rw::math::Vector3D<>& force,
                                     const rw::math::Vector3D<>& pos,
                                     rw::kinematics::State& state);

        /**
         * @brief Adds a impulse described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        /*
        virtual void addImpulseToPos(const rw::math::Vector3D<>& impulse,
                                     const rw::math::Vector3D<>& pos){
            // calculate the center force contribution
            _linImpulse += impulse;

            // calculate the torque contribution
            _angImpulse += cross( pos, impulse );
        }
        */

        /**
         * @brief Adds a impulse described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        /*
        virtual void addImpulseWToPosW(const rw::math::Vector3D<>& impulse,
                                       const rw::math::Vector3D<>& pos);
         */

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in world frame
         */
        /*
        virtual void addGravitationW(const rw::math::Vector3D<>& grav){
            _force += (_pTw.R() * grav) * _mass;
        };
        */

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in body frame
         */
        virtual void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            _torque = t;
        }

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in world frame
         */
        virtual void setTorqueW(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            _torque = inverse(rw::kinematics::Kinematics::worldTframe(_parent,state)).R() * t;
        }

        /**
         * @brief returns torque described in body frame
         */
        virtual rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const{
            return _torque;
        }

        /**
         * @brief returns torque described in world frame
         */
        virtual rw::math::Vector3D<> getTorqueW(rw::kinematics::State& state){
            return rw::kinematics::Kinematics::worldTframe(_parent,state).R() * _torque;
        }

        /**
         * @brief returns the transform from parent to body
         */
        virtual rw::math::Transform3D<> getPTBody(const rw::kinematics::State& state) const {
            return _mframe->getTransform(state);
        }

        /**
         * @brief set the transform of the body. The transform is described relative to parent.
         */
        virtual void setPTBody(const rw::math::Transform3D<>& pTb, rw::kinematics::State& state){
            _mframe->setTransform( pTb , state );
        }

        /**
         * @brief returns the transform from world to body
         */
        virtual rw::math::Transform3D<> getWTBody(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(_mframe, state);
        }

        /**
         * @brief return the linear velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getLinVel(const rw::kinematics::State& state) const {
        	const double *q = this->getData(state);
            return rw::math::Vector3D<>(q[0],q[1],q[2]);
        }

        /**
         * @brief return the linear velocity described in world frame
         */
        virtual rw::math::Vector3D<> getLinVelW(const rw::kinematics::State& state) const {
        	return getWTBody(state).R() * getLinVel(state);
        }

        /**
         * @brief set the linear velocity.
         */
        virtual void setLinVel(const rw::math::Vector3D<> &lvel, rw::kinematics::State& state){
        	double *q = this->getData(state);
        	q[0] = lvel[0];
        	q[1] = lvel[1];
        	q[2] = lvel[2];
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const {
            const double *q = this->getData(state);
        	rw::math::Vector3D<> v(q[3],q[4],q[5]);
        	return v;
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        virtual rw::math::Vector3D<> getAngVelW(rw::kinematics::State& state){
            return rw::kinematics::Kinematics::worldTframe(_parent, state).R() * getAngVel(state);
        }

        /**
         * @brief set the angular velocity of this rigid body.
         */
        virtual void setAngVel(const rw::math::Vector3D<> &avel, rw::kinematics::State& state);

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
            return _mass;
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
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        rw::math::InertiaMatrix<> calcInertiaTensorInv(const rw::kinematics::State& state) const;

        /**
         * @brief returns the inverse of the inertia tensor described in
         * world frame.
         */
        rw::math::InertiaMatrix<> calcInertiaTensorInvW(const rw::kinematics::State& state) const;

        /**
         * @brief returns the inverse of the inertia tensor described in
         * parent frame.
         */
        rw::math::InertiaMatrix<> calcInertiaTensor(const rw::kinematics::State& state) const;


        rw::kinematics::MovableFrame* getMovableFrame(){
        	return _mframe;
        }

        /**
         *
         * @note Total energy = 1/2 * mv^2 + 1/2 * Iw^2 + mgz
         */
        double calcEnergy(const rw::kinematics::State& state);

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

        rw::math::InertiaMatrix<> calcEffectiveInertia(const rw::kinematics::State& state) const;

        rw::math::InertiaMatrix<> calcEffectiveInertiaInv(const rw::kinematics::State& state) const;


    protected:
        // const variables
        const double _mass, _massInv;

        rw::kinematics::MovableFrame *_mframe;
        rw::kinematics::Frame *_parent;

        int _bodyType;

        //
        rw::math::InertiaMatrix<> _Ibody, _IbodyInv;

        // state variables
        rw::math::InertiaMatrix<> _ITensorInv,_ITensor; // inverse inertia tensor in parent frame

        rw::math::Vector3D<> _force, _forceRB, // accumulated force in parent frame
                             _torque, _torqueRB; // accumulated torque in parent frame


    };
    //! @}
} // namespace dynamics
}

#endif /*DYNAMICS_RIGIDBODY_HPP_*/
