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

#ifndef RWSIM_SIMULATOR_ODEBODY_HPP_
#define RWSIM_SIMULATOR_ODEBODY_HPP_

#include <ode/ode.h>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidJoint.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rw/math/Vector3D.hpp>
#include "ODEUtil.hpp"

namespace rwsim {
namespace simulator {
	/**
	 * @brief a convienience class for bridging RWSim body states with ode body
	 * states. Properties of the objects such as MaterialID, collision reduction
	 * threshold is kept per instance.
	 */
	class ODEBody {

	public:
		typedef enum{FIXED, KINEMATIC, RIGID, RIGIDJOINT, KINJOINT} ODEBodyType;

		/**
		 * @brief constructor for rigid bodies
		 * @param odeBody
		 * @param rwbody
		 * @param offset [in] offset of the center of mass relative to \b rwbody
		 * @return
		 */
		ODEBody(dBodyID odeBody,
				dynamics::RigidBody* rwbody,
				rw::math::Vector3D<> offset,
				int matID, int conID);

		/**
		 * @brief constructor for rigid bodies
		 * @param odeBody
		 * @param rwbody
		 * @param offset [in] offset of the center of mass relative to \b rwbody
		 * @return
		 */
		ODEBody(dBodyID odeBody,
				dynamics::RigidJoint* rwbody,
				rw::math::Vector3D<> offset,
				int matID, int conID);

		/**
		 * @brief constructor for kinematic bodies
		 * @param odeBody [in]
		 * @param rwbody
		 * @param offset [in] offset of the center of mass relative to \b rwbody
		 * @return
		 */
		ODEBody(dBodyID odeBody, dynamics::KinematicBody* rwbody,int matID, int conID);

		/**
		 * @brief constructor for fixed bodies
		 * @param geomId
		 * @param frame
		 * @return
		 */
		ODEBody(dGeomID geomId, rw::kinematics::Frame* frame,int matID, int conID);

		/**
		 * @brief destructor
		 * @return
		 */
		virtual ~ODEBody(){};

		/**
		 * @brief Called before collision checking and time stepping
		 * @param state
		 */
		void update(double dt, rw::kinematics::State& state);

		/**
		 * @brief This method updates the \b state with state info of this ode object.
		 * Which means that ode states are converted to rw states.
		 * @param state
		 */
		void postupdate(rw::kinematics::State& state);

		/**
		 * @brief resets the ODE body to the values of the RW body
		 * @param state
		 */
		void reset(const rw::kinematics::State& state);

		/**
		 * @brief returns the
		 * @return
		 */
		dynamics::Body* getRwBody(){

			return _rwBody;
		}

		dBodyID getODEBody(){
			return _bodyId;
		}

		rw::math::Transform3D<> getTransform(){
			if( _bodyId == 0)
				return rw::math::Transform3D<>::identity();
			rw::math::Transform3D<> wTb = ODEUtil::getODEBodyT3D(_bodyId);
			wTb.P() -= wTb.R()*_offset;
			return wTb;
		}

		ODEBodyType getType(){
			return _type;
		}

		int getMaterialID(){
			return _materialID;
		}

		int getContactID(){
			return _contactID;
		}

		rw::kinematics::Frame* getFrame(){
			return _rwframe;
		}

		void setCRTThres(double crtthres){
			_contactReductionThreshold = crtthres;
		}

		double getCRThres(){
			return _contactReductionThreshold;
		}

	private:
		// for rigid body
		dBodyID _bodyId;
		dJointID _jointId;
		dynamics::RigidBody *_rwBody;
		rw::kinematics::MovableFrame *_mframe;

		dynamics::KinematicBody *_kBody;

		dynamics::Body *_body;
		// the type of this body
		ODEBodyType _type;

		dGeomID _geomId;
		rw::kinematics::Frame *_rwframe;

		rw::math::Vector3D<> _offset;



		int _materialID, _contactID;
		double _contactReductionThreshold;
	};
}
}
#endif /* ODEBODY_HPP_ */
