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
#include <rwlibs/simulation/Simulator.hpp>
#include <rw/math/Vector3D.hpp>
#include "ODEUtil.hpp"

namespace rwsim {
namespace simulator {
    class ODESimulator;
	/**
	 * @brief a convienience class for bridging RWSim body states with ode body
	 * states. Properties of the objects such as MaterialID, collision reduction
	 * threshold is kept per instance.
	 */
	class ODEBody {

	public:
        /**
         * this enum determines how and what the ODEBody maps from (ODE) and into (RW).
         */
		typedef enum{FIXED //! a fixed body
		            , KINEMATIC //! a kinematic ode body and a kinematic RW body
		            , RIGID //! a rigid ode body and a RigidBody RW
		            , RIGIDODE //! a rigid ode body and a RW Body type (any other than RigidBody)
		            , LINK} ODEBodyType;

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
		 * @brief constructor for user defined type, eg. body does not necesarilly need to by
		 * of RigidBody to create an ODEBody::RIGID
		 * @param odeBody
		 * @param body
		 * @param offset
		 * @param matID
		 * @param conID
		 * @param type
		 */
        ODEBody(dBodyID odeBody,
                dynamics::Body* body,
                rw::math::Vector3D<> offset,
                int matID, int conID,
                ODEBodyType type);

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
		ODEBody(std::vector<dGeomID> geomId, dynamics::Body* body, int matID, int conID);

		/**
		 * @brief destructor
		 * @return
		 */
		virtual ~ODEBody(){};

		/**
		 * @brief Called before collision checking and time stepping
		 * @param state
		 */
		void update(const rwlibs::simulation::Simulator::UpdateInfo& dt, rw::kinematics::State& state);

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

		 //! returns the RobWork Body
		dynamics::Body* getRwBody(){ return _body; }

		//! get the ODE bodyId
		dBodyID getBodyID(){ return _bodyId; }

		//! get transform from world to bodyframe
		rw::math::Transform3D<> getTransform();

		//! get transform from world to COM
		rw::math::Transform3D<> getTransformCOM();

        //! set transform of body, using bodyframe
        void setTransform(const rw::kinematics::State& state);

        //! set transform of body, using bodyframe
		void setTransform(const rw::math::Transform3D<>& wTbody);

		//! set transform of body, using COM
		void setTransformCOM(const rw::math::Transform3D<>& wTcom);

		//! get type of ODEBody
		ODEBodyType getType(){return _type; }

		//! get material id
		int getMaterialID(){ return _materialID; }

		//! get contact model id
		int getContactID(){ return _contactID; }

		//! get body frame
		rw::kinematics::Frame* getFrame(){ return _rwframe; }

		//! set contact reduction parameter
		void setCRTThres(double crtthres){ _contactReductionThreshold = crtthres; }

		//! get contact reduction parameter
		double getCRThres(){ return _contactReductionThreshold; }

		rw::math::Vector3D<> getLastForce(){ return _lastForce; }

		void bodyChangedListener(dynamics::Body::BodyEventType eventtype);

		void setTriGeomData(std::vector<ODEUtil::TriGeomData*>& data){ _triGeomDatas = data; }
		std::vector<ODEUtil::TriGeomData*>& getTriGeomData(){ return _triGeomDatas; }

		static ODEBody* makeRigidBody(dynamics::Body* rwbody,  dSpaceID spaceId, ODESimulator *sim);
		static ODEBody* makeKinematicBody(dynamics::Body* kbody, dSpaceID spaceid, ODESimulator *sim);
		static ODEBody* makeFixedBody(dynamics::Body* kbody, dSpaceID spaceid, ODESimulator *sim);
	private:

		// for rigid body
        rw::kinematics::MovableFrame *_mframe;

        dBodyID _bodyId;
        dynamics::Body *_body;
        rw::kinematics::Frame *_rwframe;
        ODEBodyType _type;

        double _contactReductionThreshold;
        int _materialID, _contactID;

        dGeomID _geomId;
        std::vector<dGeomID> _geomIds;
        std::vector<ODEUtil::TriGeomData*> _triGeomDatas;

        dynamics::RigidBody *_rwBody;
        dynamics::KinematicBody *_kBody;

        rw::math::Vector3D<> _offset;
		rw::math::Vector3D<> _lastForce;
	};
}
}
#endif /* ODEBODY_HPP_ */
