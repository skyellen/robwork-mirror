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

#ifndef RWSIM_DYNAMICS_RIGIDJOINT_HPP_
#define RWSIM_DYNAMICS_RIGIDJOINT_HPP_

#ifdef zkdaslkdiasdn
#include <rw/math/Vector3D.hpp>
#include <rw/math/InertiaMatrix.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/models/Joint.hpp>

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
     */
    class RigidJoint : public Body
    {
    public:

        RigidJoint(
            const BodyInfo& info,
            rw::models::Joint* joint,
            rw::models::Object::Ptr obj,
            rw::kinematics::State& state
            );

        virtual ~RigidJoint(){};

    public: // functions that need to be implemented by specialized class

        /**
         * @copydoc Body::getPointVelW
         */
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p,const rw::kinematics::State &state) const;

        /**
         * @copydoc Body::getEffectiveMassW
         */
        rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc);

        /**
         * @copydoc Body::resetState
         */
        void reset(rw::kinematics::State &state);

        double calcEnergy(const rw::kinematics::State &state){ return 0;};

    public:

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
         */
        virtual void setForceW(const rw::math::Vector3D<>& f){
            _force = _pTw.R() * f;
        }

        /**
         * @brief Gets the force described in world frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForceW(){
            return _wTp.R() * _force;
        }

        /**
         * @brief Gets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const {
            return _force;
        }

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state){
            _force += force;
        }

        /**
         * @brief Adds a force described in world frame to the
         * center of mass of this body
         */
        virtual void addForceW(const rw::math::Vector3D<>& force){
            _force += _pTw.R() * force;
        }

        /**
         * @brief Adds a force described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        virtual void addForceToPos(const rw::math::Vector3D<>& force,
                                   const rw::math::Vector3D<>& pos){
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
                                     const rw::math::Vector3D<>& pos);

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in body frame
         */
        virtual void addGravitation(const rw::math::Vector3D<>& grav){
            _force += grav * _mass;
        };

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in world frame
         */
        virtual void addGravitationW(const rw::math::Vector3D<>& grav){
            _force += (_pTw.R() * grav) * _mass;
        };

        /**
         * @brief sets gravitation to the body where the gravitation is
         * described in body frame
         */
        virtual void setGravitation(const rw::math::Vector3D<>& grav){
            _force = grav * _mass;
        };

        /**
         * @brief sets gravitation to the body where the gravitation is
         * described in world frame
         */
        virtual void setGravitationW(const rw::math::Vector3D<>& grav){
            _force = (_pTw.R() * grav) * _mass;
        };

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
        virtual void setTorqueW(const rw::math::Vector3D<>& t){
            _torque = _pTw.R() * t;
        }

        virtual void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            _torque += t;
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
        virtual rw::math::Vector3D<> getTorqueW(){
            return _wTp.R() * _torque;
        }

        /**
         * @brief returns the transform from parent to body
         */
        virtual const rw::math::Transform3D<>& getPTBody(){
            return _pTb;
        }

        /**
         * @brief returns the transform from world to body
         */
        virtual const rw::math::Transform3D<>& getWTBody(){
            return _wTb;
        }

        /**
         * @brief return the linear velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getLinVel(){
            return _linVel;
        }

        /**
         * @brief return the linear velocity described in world frame
         */
        virtual rw::math::Vector3D<> getLinVelW(){
            return _wTp.R() * _linVel;
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        virtual const rw::math::Vector3D<>& getAngVel(){
            return _angVel;
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        virtual rw::math::Vector3D<> getAngVelW(){
            return _wTp.R() * _angVel;
        }

        /**
         * @brief calculates the relative velocity in parent frame of a point p on the body
         * described in parent frame.
         */
        rw::math::Vector3D<> getPointVel(const rw::math::Vector3D<>& p){
            return _linVel + cross(_angVel, p);
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
        inline double getInvMass() const {
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
        const rw::math::InertiaMatrix<>& getInertiaTensorInv() const {
            return _IInv;
        }

        /**
         * @brief returns the inverse of the inertia tensor described in
         * world frame.
         */
        rw::math::InertiaMatrix<> getInertiaTensorInvW() const {
            return _wTb.R() * ( _IbodyInv * inverse( _wTb.R() ) );
        }


        rw::models::Joint* getJoint(){
        	return _joint;
        }

        rw::kinematics::Frame& getFrame(){
          return *_frame;
        }


    protected:
        // const variables
        const double _mass,_massInv;

        rw::models::Joint *_joint;
        rw::kinematics::Frame *_frame;
        //BodyIntegrator *_integrator;

        std::string _materialID;

        int _bodyType;

        rw::math::InertiaMatrix<> _Ibody,
                                  _IbodyInv;

        // state variables
        rw::math::Transform3D<> _pTb; // position and orientation

        rw::math::InertiaMatrix<> _IInv; // inverse inertia tensor in parent frame

        rw::math::Vector3D<> _force, _forceRB, // accumulated force in parent frame
                             _torque, _torqueRB; // accumulated torque in parent frame

        rw::math::Vector3D<> _linVel, _linVelRB, // linear velocity in parent frame
                             _angVel, _angVelRB; // angular velocity in parent frame

        rw::math::Transform3D<> _wTb, // world to body
                                _bTw,// body to world
                                _pTw, // parent to world
                                _wTp; // world to parent

        // aux variables


    };
    //! @}
} // namespace dynamics
}

#endif

#endif /*DYNAMICS_RigidJoint_HPP_*/
