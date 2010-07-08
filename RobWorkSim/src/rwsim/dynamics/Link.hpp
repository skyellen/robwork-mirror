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
	 * @brief The Link is a body that is part of a dynamic device where joints are used to
	 * constrain the movement of links.
	 *
	 * As such the Link has no state
	 * variables and relies purely on the state of the dynamic device to provide its
	 * velocity and acceleration.
	 */
    class Link : public Body
    {
    public:
        Link( const BodyInfo& info,
                   rw::kinematics::Frame *base,
                   std::vector<Link*> parents,
                   rw::models::Device &dev,
                   rw::models::Joint &j,
                   const std::vector<rw::kinematics::Frame*>& frames,
                   rw::kinematics::State &state);

    	virtual ~Link();

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
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& wPp);

        /**
         * @copydoc Body::resetState
         */
        void resetState(rw::kinematics::State &state);

        void calcAuxVarialbles(rw::kinematics::State& state){}

        double calcEnergy() const {return 0;};
    public:

    	rw::math::InertiaMatrix<> getEffectiveMass();

    	void addQd( double vel );

    	void addTorque( double t );

    	double getQd(){
    		return _vel;
    	}

        void setAcc(double acc){
        	_acc = acc;
        }

        const std::string& getMaterial(){
            return _materialID;
        }

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force){
            _force += force;
        }

        /**
         * @brief Adds a force described in world frame to the
         * center of mass of this body
         */
        virtual void addForceW(const rw::math::Vector3D<>& force){
            _force += _bTw.R() * force;
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
         * @brief Adds a impulse described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        virtual void addImpulseToPos(const rw::math::Vector3D<>& impulse,
                                     const rw::math::Vector3D<>& pos){
            // calculate the center force contribution
            _linImpulse += impulse;

            // calculate the torque contribution
            _angImpulse += cross( pos, impulse );
        }

        /**
         * @brief Adds a impulse described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        virtual void addImpulseWToPosW(const rw::math::Vector3D<>& impulse,
                                       const rw::math::Vector3D<>& pos);

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in body frame
         */
        virtual void addGravitation(const rw::math::Vector3D<>& grav){
            //_force += grav * _mass;

        };

        /**
         * @brief adds gravitation to the body where the gravitation is
         * described in world frame
         */
        virtual void addGravitationW(const rw::math::Vector3D<>& grav){
            //_force += (_pTw.R() * grav) * _mass;
        };

        /**
         * @brief
         */
        void addChild(Link *child){
        	_children.push_back(child);
        }

        /**
         * @brief get the body inertia of the link
         */
        rw::math::InertiaMatrix<> getInertia(){
        	return rw::math::InertiaMatrix<>(1,1,1);
        }

        void setTargetVel(double vel){
        	_targetVel = vel;
        }
        /*
        void setAcc(double acc){
        	_maxAcc = acc;
        }*/

        rw::models::Joint *getJoint(){
            return &_jointFrame;
        }

    private:

    	std::string _materialID;

    	std::vector<Link*> _children,_parents;

    	rw::models::Joint &_jointFrame;

    	rw::models::Device &_dev;

    	rw::math::Jacobian _jac, _jacRB;

    	rw::math::Vector3D<> _force, _forceRB, // accumulated force in parent frame
                             _torque, _torqueRB; // accumulated torque in parent frame

        rw::math::Vector3D<> _linImpulse, _linImpulseRB, // linear impulse in parent frame
                             _angImpulse, _angImpulseRB; // angular impulse in parent frame

        rw::math::Transform3D<> _wTb,_wTbRB, // world to body
                                _bTw,_bTwRB; // body to world

        rw::math::Transform3D<> _wTbase, _baseTw; // world to base
        rw::math::Transform3D<> _baseTb, _bTbase;

        rw::math::Transform3D<> _baseTbRB;
        rw::math::Transform3D<> _bTbaseRB;

        double _vel,_pos,_acc,_posRB, _velRB;

        double _targetVel;

        rw::kinematics::Frame *_base;

        int _impulseIterations;

        size_t _jointNr;


    };
    //! @}
}
}

#endif /*LINK_HPP_*/
