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

#ifndef RWSIM_SIMULATOR_ODEJOINT_HPP_
#define RWSIM_SIMULATOR_ODEJOINT_HPP_

#include <ode/ode.h>
#include <rwsim/dynamics/Body.hpp>
//#include <rwsim/dynamics/RigidJoint.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/models/Joint.hpp>
#include "ODEBody.hpp"

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {
    class ODESimulator;

	/**
	 * @brief this class bridges ODE's joints with RobWork joints. The joint is a
	 * pure constraint type and hence does not have any geometry. It does however
	 * constraint two geometrical bodies together.
	 */
	class ODEJoint {
	public:
		typedef enum{FIXED, RIGID, DEPEND, DEPEND_PAR} ODEJointType;
		typedef enum{Revolute, Prismatic} JointType;

		/**
		 * @brief constructs a ODE joint based on a robwork joint.
		 * @param rwjoint
		 * @param parent
		 * @param child
		 * @param sim
		 * @param state
		 */
        ODEJoint(rw::models::Joint* rwjoint,
                 ODEBody* parent,
                 ODEBody* child,
                 ODESimulator *sim,
                 const rw::kinematics::State& state);

        /*
		ODEJoint(JointType jtype,
				 dJointID odeJoint,
				 dJointID odeMotor,
				 dBodyID body,
				 dynamics::RigidJoint* rwbody);

		ODEJoint(JointType jtype,
				 dJointID odeJoint,
				 dJointID odeMotor,
				 dBodyID body,
				 ODEJoint* owner,
				 rw::kinematics::Frame *bframe,
				 double scale, double off,
				 dynamics::RigidJoint* rwjoint);
        */

		virtual ~ODEJoint(){};

        void setForce(double vel){
        	//if(isMotorEnabled())
        	//	RW_WARN("PLEASE DISABLE MOTOR!");

            //if(_jtype==Revolute) dJointAddHingeTorque(_jointId, vel);
            //else dJointAddSliderForce(_jointId, vel);
        	if(_jtype==Revolute)
        		dJointAddAMotorTorques(_motorId, vel,vel,vel);
        }

        double getActualVelocity(){
            if(_jtype==Revolute)  return dJointGetHingeAngleRate( _jointId );
            return dJointGetSliderPositionRate ( _jointId );
        }

        double getAngle(){
            double val;
            if(_jtype==Revolute) val = dJointGetHingeAngle( _jointId );
            else val = dJointGetSliderPosition ( _jointId );
            return val;
        }

        ODEJoint* getOwner(){
            return _owner;
        }

        ODEJointType getType(){
            return _type;
        }

        void reset(const rw::kinematics::State& state);

        double getScale(){ return _scale; };
        double getOffset(){ return _off; };


        ODEBody* getParent(){ return _parent; }

        ODEBody* getChild() { return _child; }

        rw::models::Joint* getJoint(){
            return _rwJoint;
        }

        bool isDepend(){ return getType()==DEPEND || getType()==DEPEND_PAR;}

        /// Functions that require a MOTOR

        void setVelocity(double vel){
            if(_jtype==Revolute) dJointSetAMotorParam(_motorId, dParamVel, vel);
            else dJointSetLMotorParam(_motorId, dParamVel, vel);
        }

		double getVelocity(){
			double vel;
			if(_jtype==Revolute) vel = dJointGetAMotorParam(_motorId, dParamVel);
			else vel = dJointGetLMotorParam(_motorId, dParamVel);
			return vel;
		}

		void setMaxForce(double force){
			if(force>0.0)
				_disableForceTmp = force;
			if(_jtype==Revolute) dJointSetAMotorParam(_motorId,dParamFMax, force );
			else dJointSetLMotorParam(_motorId,dParamFMax, force );
		}

		double getMaxForce(){
			if(_jtype==Revolute)return dJointGetAMotorParam(_motorId,dParamFMax);
			return dJointGetLMotorParam(_motorId,dParamFMax);
		}

        void setAngle(double pos){
          if(_jtype==Revolute) dJointSetAMotorAngle(_motorId, 0, pos);
            //else dJointSetLMotorAngle(_motorId, 0, pos);
        }

        void setMotorEnabled(bool enabled){
        	std::cout << "set motor " << enabled << " " << _rwJoint->getName() << " id: "<<_motorId << "\n";
            if(enabled){
            	//dJointEnable(_motorId);
            	setMaxForce(_disableForceTmp);
            } else {
            	_disableForceTmp = getMaxForce();
            	setMaxForce(0.0);
            	//dJointDisable(_motorId);
            }
        }

        bool isMotorEnabled(){
            if(dJointIsEnabled (_motorId))
                return true;
            return false;
        }

		// renamed to getChild
		//dBodyID getODEBody(){
		//	return _bodyId;
		//}

	private:
		//dBodyID _bodyId;
		dJointID _jointId, _motorId;
		rw::models::Joint *_rwJoint;
		//dynamics::RigidJoint *_rwJoint;

		ODEJoint *_owner;
		double _scale,_off;
		ODEJointType _type;
		JointType _jtype;

		//rw::kinematics::Frame *_bodyFrame;
		rw::math::Vector3D<> _offset;
		ODEBody *_parent, *_child;
		double _disableForceTmp;

	};
}
}

#endif /* ODEBODY_HPP_ */
