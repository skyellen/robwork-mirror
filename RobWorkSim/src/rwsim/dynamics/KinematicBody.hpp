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
	 * @brief a kinematic body
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
         * @copydoc Body::updateVelocity
         */
        virtual void updateVelocity(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::updatePosition
         */
        virtual void updatePosition(double h, rw::kinematics::State& state);

        /**
         * @copydoc Body::updateImpulse
         */
        virtual void updateImpulse();

        /**
         * @copydoc Body::getPointVelW
         */
        rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& wPp);

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

        void calcAuxVarialbles(rw::kinematics::State& state){}

        double calcEnergy(){return 0;};
    public:

        const std::string& getMaterial(){
            return _materialID;
        }

        /**
         * @brief get the body inertia of the link
         */
        rw::math::InertiaMatrix<> getInertia(){
        	return rw::math::InertiaMatrix<>(1,1,1);
        }

        rw::models::Joint* getJoint(){
            return _jointFrame;
        }

    private:

    	std::string _materialID;

    	rw::models::Joint *_jointFrame;

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
