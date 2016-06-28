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

#include "ODESimulator.hpp"

#include <ode/ode.h>

#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <rw/models/JointDevice.hpp>
//#include <rw/models/RevoluteJoint.hpp>
//#include <rw/models/PrismaticJoint.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <boost/foreach.hpp>

#include <rw/kinematics/FramePairMap.hpp>

//#include <rw/models/DependentRevoluteJoint.hpp>
//#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/common/TimerUtil.hpp>

#include "ODEConstraint.hpp"
#include "ODEKinematicDevice.hpp"
#include "ODEVelocityDevice.hpp"
#include "ODEDebugRender.hpp"
#include "ODEUtil.hpp"
#include "ODESuctionCupDevice.hpp"
#include "ODEMaterialMap.hpp"

#include <rwsim/dynamics/OBRManifold.hpp>

#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include <rw/common/Log.hpp>

#include <boost/bind.hpp>

#include <sstream>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::sensor;
using namespace rwsim;

using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::models;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::common;

using namespace rwlibs::simulation;
using namespace rwlibs::proximitystrategies;

#define INITIAL_MAX_CONTACTS 1000

#define RW_DEBUGS( str ) rw::common::Log::debugLog() << str  << std::endl;
//#define RW_DEBUGS( str )

/*
#define TIMING( str, func ) \
    { long start = rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }
*/
#define TIMING( str, func ) {func;}

namespace {

	std::string printArray(const dReal* arr, int n){
	    std::stringstream str;
	    str << "(";
	    for(int i=0; i<n-1; i++){
	        str << arr[i]<<",";
	    }
	    str << arr[n-1]<<")";
	    return str.str();
	}

	void printMassInfo(const dMass& dmass, const Frame& frame ){
	    std::cout  << "----- Mass properties for frame: " << frame.getName() << std::endl;
	    std::cout  << "- Mass    : " << dmass.mass << std::endl;
	    std::cout  << "- Center  : " << printArray(&(dmass.c[0]), 3) << std::endl;
	    std::cout  << "- Inertia : " << printArray(&dmass.I[0], 3) << std::endl;
	    std::cout  << "-           " << printArray(&dmass.I[3], 3) <<  std::endl;
	    std::cout  << "-           " << printArray(&dmass.I[6], 3) << std::endl;
	    std::cout  << "----------------------------------------------------" << std::endl;
	}

	void nearCallback(void *data, dGeomID o1, dGeomID o2)
	{
		if ( (dGeomIsSpace (o1) && !dGeomIsSpace (o2)) ||
	          (!dGeomIsSpace (o1) && dGeomIsSpace (o2))
	    ) {
	              // colliding a space with something
	              dSpaceCollide2 (o1,o2,data,&nearCallback);
	              // collide all geoms internal to the space(s)
	              //if (dGeomIsSpace (o1)) dSpaceCollide ((dSpaceID)o1,data,&nearCallback);
	              //if (dGeomIsSpace (o2)) dSpaceCollide ((dSpaceID)o2,data,&nearCallback);
	    } else {
	        reinterpret_cast<ODESimulator*>(data)->handleCollisionBetween(o1,o2);

	    }
	}
}


//const bool ODESimulator::ODERegistrered = rwsim::simulator::PhysicsEngineFactory::Register<ODESimulator>::_Register("ODE");

bool isInErrorGlobal = false;
bool badLCPSolution = false;

//const double CONTACT_SURFACE_LAYER = 0.0001;
//const double MAX_SEP_DISTANCE = 0.0005;
//const double MAX_PENETRATION  = 0.00045;

//const int CONTACT_SURFACE_LAYER = 0.006;
//const double MAX_SEP_DISTANCE = 0.008;
//const double MAX_PENETRATION  = 0.007;


double ODESimulator::getMaxSeperatingDistance(){
    return _maxSepDistance;
}

ODESimulator::ODESimulator(DynamicWorkCell::Ptr dwc, rwsim::contacts::ContactDetector::Ptr detector):
	_dwc(dwc),
	_time(0.0),
	_render(ownedPtr( new ODEDebugRender(this) ) ),
    _contacts(INITIAL_MAX_CONTACTS),
    _filteredContacts(INITIAL_MAX_CONTACTS+10),
    _rwcontacts(INITIAL_MAX_CONTACTS),
    _rwClusteredContacts(INITIAL_MAX_CONTACTS+10),
    _srcIdx(INITIAL_MAX_CONTACTS+10),
    _dstIdx(INITIAL_MAX_CONTACTS+10),
    _nrOfCon(0),
    _enabledMap(20,1),
    _materialMap(dwc->getMaterialData()),
    _contactMap(dwc->getContactData()),
    _narrowStrategy(new ProximityStrategyPQP()),
    _sensorFeedbacks(5000),
    _sensorFeedbacksGlobal(100),
    _nextFeedbackIdx(0),
    _nextFeedbackGlobalIdx(0),
    _excludeMap(0,100),
    _oldTime(0),
    _useRobWorkContactGeneration(true),
    _prevStepEndedInCollision(false),
    _detector(detector),
    _logContactingBodies(false)
{

    // verify that the linked ode library has the correct
    int isDouble = dCheckConfiguration ("ODE_double_precision");
    //int isRelease = dCheckConfiguration ("ODE_EXT_no_debug");

    if(sizeof(dReal)==sizeof(double)){
        if(isDouble==0){
            RW_THROW("Current linked library does not support double! change dDOUBLE to dSINGLE or link agains ode lib with double support!");
        }
    } else {
        if(isDouble==1){
            RW_THROW("Current linked library does not support single! change dSINGLE to dDOUBLE or link agains ode lib with single support!");
        }
    }

    // setup DWC changed event
    if(_dwc)
        _dwc->changedEvent().add( boost::bind(&ODESimulator::DWCChangedListener, this, _1, _2), this);
}

ODESimulator::ODESimulator():
    _dwc(NULL),
    _time(0.0),
    _render(ownedPtr( new ODEDebugRender(this) ) ),
    _contacts(INITIAL_MAX_CONTACTS),
    _filteredContacts(INITIAL_MAX_CONTACTS+10),
    _rwcontacts(INITIAL_MAX_CONTACTS),
    _rwClusteredContacts(INITIAL_MAX_CONTACTS+10),
    _srcIdx(INITIAL_MAX_CONTACTS+10),
    _dstIdx(INITIAL_MAX_CONTACTS+10),
    _nrOfCon(0),
    _enabledMap(20,1),
    //_materialMap(dwc->getMaterialData()),
    //_contactMap(dwc->getContactData()),
    _narrowStrategy(new ProximityStrategyPQP()),
    _sensorFeedbacks(5000),
    _sensorFeedbacksGlobal(100),
    _nextFeedbackIdx(0),
    _nextFeedbackGlobalIdx(0),
    _excludeMap(0,100),
    _oldTime(0),
    _useRobWorkContactGeneration(true),
    _prevStepEndedInCollision(false),
    _detector(NULL),
    _logContactingBodies(false)
{

    // verify that the linked ode library has the correct
    int isDouble = dCheckConfiguration ("ODE_double_precision");
    //int isRelease = dCheckConfiguration ("ODE_EXT_no_debug");

    if(sizeof(dReal)==sizeof(double)){
        if(isDouble==0){
            RW_THROW("Current linked library does not support double! change dDOUBLE to dSINGLE or link agains ode lib with double support!");
        }
    } else {
        if(isDouble==1){
            RW_THROW("Current linked library does not support single! change dSINGLE to dDOUBLE or link agains ode lib with single support!");
        }
    }

    // setup DWC changed event
    if(_dwc)
        _dwc->changedEvent().add( boost::bind(&ODESimulator::DWCChangedListener, this, _1, _2), this);
}

ODESimulator::~ODESimulator() {
	delete _narrowStrategy;
}

void ODESimulator::load(rwsim::dynamics::DynamicWorkCell::Ptr dwc){
    _dwc = dwc;
    _materialMap = dwc->getMaterialData();
    _contactMap = dwc->getContactData();
    _dwc->changedEvent().add( boost::bind(&ODESimulator::DWCChangedListener, this, _1, _2), this);
}


bool ODESimulator::setContactDetector(rwsim::contacts::ContactDetector::Ptr detector) {
	_detector = detector;
	return true;
}

void ODESimulator::DWCChangedListener(DynamicWorkCell::DWCEventType type, boost::any data){
    std::cout << "DWC changed, type: " << type << std::endl;
    // TODO: handle all the event types
    if( type==DynamicWorkCell::GravityChangedEvent ) {
        Vector3D<> gravity = _dwc->getGravity();
        dWorldSetGravity ( _worldId, gravity(0), gravity(1), gravity(2) );
    }

}


void ODESimulator::setEnabled(Body::Ptr body, bool enabled){
    if(!body)
        RW_THROW("Body is NULL!");
    ODEBody *odebody = _rwFrameToODEBody[ body->getBodyFrame() ];
    Frame *frame = _rwODEBodyToFrame[ odebody ];


    if( enabled ) {
        dBodyEnable(odebody->getBodyID());
        _enabledMap[*frame] = 1;
    } else{
        dBodyDisable(odebody->getBodyID());
        _enabledMap[*frame] = 0;
    }

}

void ODESimulator::setDynamicsEnabled(dynamics::Body::Ptr body, bool enabled){
    ODEBody *odebody = _rwFrameToODEBody[ body->getBodyFrame() ];
	//std::cout << "1" << std::endl;
    if(odebody==NULL)
        return;
	//std::cout << "2" << std::endl;
    //if(odebody->getType()!=ODEBody::RIGID || odebody->getType()!=ODEBody::RIGIDJOINT)
    //    return;
	//std::cout << "3" << std::endl;
    if(enabled){
        //std::cout << "SET DYNAMIC BODY" << std::endl;
		dBodySetDynamic( odebody->getBodyID() );
	} else {
		//std::cout << "SET KINEMATIC BODY" << std::endl;
        dBodySetKinematic( odebody->getBodyID() );
	}
}

namespace {
	bool isClose(dReal *m1, const dReal *P, const dReal *R, double eps ){
		double rsum = 0;
		for(int i=0;i<12;i++){
			float val = (float)fabs(m1[i]-R[i]);
			rsum += val*val;
		}
		double psum = fabs(m1[12]-P[0])*fabs(m1[12]-P[0])+
					  fabs(m1[14]-P[2])*fabs(m1[14]-P[2]);

		return rsum<eps && psum<eps;
	}

    void drealCopy(const dReal *src, dReal *dst, int n){
        for(int i=0;i<n;i++)
            dst[i]=src[i];
    }

}


/*
    *  Reset each body's position - dBody[Get,Set]Position()
    * Reset each body's quaternion - dBody[Get,Set]Quaternion() ODE stores rotation in quaternions, so don't save/restore in euler angles because it will have to convert to/from quaternions and you won't get a perfect restoration.
    * Reset each body's linear velocity - dBody[Get,Set]LinearVel()
    * Reset each body's angular velocity - dBody[Get,Set]AngularVel()
    * In the quickstep solver, "warm starting" is enabled by default. This involves storing 6 dReal lambda values with each joint so that the previous solution can be applied towards achieving the next. These values must be stored and reset or warm starting disabled. There is no current method to retrieve or set these values, so one must be added manually. They are found at the bottom of the dxJoint struct in joint.h in the source. To disable warm starting, comment out the "#define WARM_STARTING" line in quickstep.cpp.
    * Reset "desired velocity" and "FMax" parameters for motorized joints
    * Reset the "enable" state of every body - If bodies are set to auto-disable, you may need to reset their associated variables (adis_timeleft,adis_stepsleft,etc) - there is no current api method for that.
    * Remove contact joints created during the previous step
    * You might want to assert that the force and torque accumulators are zero
    * Make sure the rest of your controller/simulation is also reset
 */



void ODESimulator::saveODEState(){
    _odeStateStuff.clear();
	// first run through all rigid bodies and set the velocity and force to zero
	// std::cout  << "- Resetting bodies: " << _bodies.size() << std::endl;


    BOOST_FOREACH(dBodyID odebody, _allbodies){
    //BOOST_FOREACH(ODEBody *ob, _odeBodies){
        //if(ob->getType()==ODEBody::FIXED)
        //    continue;
        //dBodyID odebody = ob->getBodyID();
		ODEStateStuff res;
		res.body = odebody;
		dBodyID body = odebody;
		drealCopy( dBodyGetPosition(body), res.pos, 3);
		drealCopy( dBodyGetQuaternion(body), res.rot, 4);
		drealCopy( dBodyGetLinearVel  (body), res.lvel, 3);
		drealCopy( dBodyGetAngularVel (body), res.avel, 3);
		drealCopy( dBodyGetForce  (body), res.force, 3);
		drealCopy( dBodyGetTorque (body), res.torque, 3);
		_odeStateStuff.push_back(res);
	}

	/*
	BOOST_FOREACH(dJointID joint, _alljoints){
	    ODEStateStuff res;
	    res.joint = joint;
	    // test what joint type it is
	    dJointType type = dJointGetType(joint);
	    if( type == dHingeJoint ){
	        dJointGetHingeAngle(joint);
	    }


	    drealCopy( dBodyGetPosition(body), res.pos, 3);
    }
*/

	BOOST_FOREACH(ODEJoint* joint, _allODEJoints){
		ODEStateStuff res;
		res.joint = joint;
		//res.desvel = joint->getVelocity();
		//res.fmax = joint->getMaxForce();
		_odeStateStuff.push_back(res);
	}

}

void ODESimulator::restoreODEState(){
	BOOST_FOREACH(ODEStateStuff &res, _odeStateStuff){
		if(res.body!=NULL){
			dBodySetPosition(res.body, res.pos[0], res.pos[1], res.pos[2]);
			dBodySetQuaternion(res.body, res.rot);
			dBodySetLinearVel(res.body, res.lvel[0], res.lvel[1], res.lvel[2]);
			dBodySetAngularVel(res.body, res.avel[0], res.avel[1], res.avel[2]);
			dBodySetForce(res.body, res.force[0], res.force[1], res.force[2]);
			dBodySetTorque (res.body, res.torque[0], res.torque[1], res.torque[2]);
		} else if(res.joint!=NULL){
			//res.joint->setVelocity(res.desvel);
			//res.joint->setMaxForce(res.desvel);
		}
	}

	//BOOST_FOREACH(ODETactileSensor* sensor,_odeSensors){
	//	sensor->clear();
	//}
}

void ODESimulator::step(double dt, rw::kinematics::State& state)

{
	if(isInErrorGlobal)
		return;
	_stepState = &state;

	//std::cout << "-------------------------- STEP --------------------------------" << std::endl;
	//double dt = 0.001;
	_maxPenetration = 0;
    RW_DEBUGS("-------------------------- STEP --------------------------------");
    double lastDt = _time-_oldTime;
    if(lastDt<=0)
        lastDt = 0;

    RW_DEBUGS("------------- Collisions at " << _time << " :");
    // Detect collision
    _allcontacts.clear();
    if( _useRobWorkContactGeneration ){
    	if (_detector == NULL) {
    		TIMING("Collision: ", detectCollisionsRW(state, false) );
    	} else {
    		TIMING("Collision: ", detectCollisionsContactDetector(state) );
    	}
        {
            boost::mutex::scoped_lock lock(_contactMutex);
            _allcontactsTmp = _allcontacts;
        }
    } else {
        try {
            TIMING("Collision: ", dSpaceCollide(_spaceId, this, &nearCallback) );
            _allcontactsTmp = _allcontacts;
        } catch ( ... ) {
            std::cout << "Collision ERROR";
            Log::errorLog() << "******************** Caught exeption in collision function!*******************" << std::endl;
        }
    }

    // we roll back to this point if there is any penetrations in the scene
    RW_DEBUGS("------------- Save state:");
    saveODEState();
    State tmpState = state;
    double dttmp = dt;
    int i;
    const int MAX_TIME_ITERATIONS = 10;
    badLCPSolution = false;
    int badLCPcount = 0;
    for(i=0;i<MAX_TIME_ITERATIONS;i++){

        if(isInErrorGlobal){
            dJointGroupEmpty(_contactGroupId);
            // and the joint feedbacks that where used is also destroyed
            _nextFeedbackIdx=0;
            RW_THROW("Collision error");
        }

        RW_DEBUGS("------------- Controller update :");
        Simulator::UpdateInfo conStepInfo;
        conStepInfo.dt = dttmp;
        conStepInfo.dt_prev = lastDt;
        conStepInfo.time = _time;
        conStepInfo.rollback = i>0;
        BOOST_FOREACH(SimulatedController::Ptr controller, _controllers ){
            controller->update(conStepInfo, tmpState);
        }

        RW_DEBUGS("------------- Device pre-update:");
        BOOST_FOREACH(ODEDevice *dev, _odeDevices){
            dev->update(conStepInfo, tmpState);
        }

		RW_DEBUGS("------------- Constraints pre-update:");
		BOOST_FOREACH(ODEConstraint *constraint, _odeConstraints) {
			constraint->update(conStepInfo, tmpState);
		}

        RW_DEBUGS("------------- Body pre-update:");
        BOOST_FOREACH(ODEBody *body, _odeBodies){
            body->update(conStepInfo, tmpState);
        }

        // Step world
        RW_DEBUGS("------------- Step dt=" << dttmp <<" at " << _time << " :");

		//TIMING("Step: ", dWorldStep(_worldId, dttmp));
        /*
        std::cout << "-- time    : " << _time << "\n";
        std::cout << "-- dt orig : " << dt << "\n";
        std::cout << "-- dt div  : " << dttmp << "\n";
        std::cout << "-- step divisions: " << i << "\n";
        std::cout << "-- bad lcp count : " << badLCPcount << "\n";
        std::cout << "-- Bodies: \n";
        BOOST_FOREACH(dBodyID body, _allbodies){
            dReal vec[4];
            ODEBody *data = (ODEBody*) dBodyGetData(body);
            if(data!=NULL){
                std::cout << "--- Body: " << data->getRwBody()->getName();
            } else {
                std::cout << "--- Body: NoRWBODY";
            }
            drealCopy( dBodyGetPosition(body), vec, 3);
            std::cout << "\n---- pos   : " << printArray(vec, 3);
            drealCopy( dBodyGetQuaternion(body), vec, 4);
            std::cout << "\n---- rot   : " << printArray(vec, 4);
            drealCopy( dBodyGetLinearVel  (body), vec, 3);
            std::cout << "\n---- linvel: " << printArray(vec, 3);
            drealCopy( dBodyGetAngularVel (body), vec, 3);
            std::cout << "\n---- angvel: " << printArray(vec, 3);
            drealCopy( dBodyGetForce  (body), vec, 3);
            std::cout << "\n---- force : " << printArray(vec, 3);
            drealCopy( dBodyGetTorque (body), vec, 3);
            std::cout << "\n---- torque: " << printArray(vec, 3);
            std::cout << "\n";
        }
        std::cout << "--\n-- contacts: \n";
        BOOST_FOREACH(ContactPoint p, _allcontactsTmp){
            std::cout << "-- pos: "<< p.p << "\n";
            std::cout << "-- normal: "<< p.n << "\n";
            std::cout << "-- depth: "<< p.penetration << "\n";
        }

*/
	    try {
	        switch(_stepMethod){
	        case(WorldStep): TIMING("Step: ", dWorldStep(_worldId, dttmp)); break;
	        case(WorldQuickStep): TIMING("Step: ", dWorldQuickStep(_worldId, dttmp)); break;
	        //case(WorldFast1): TIMING("Step: ", dWorldStepFast1(_worldId, dt, _maxIter)); break;
	        default:
	            TIMING("Step: ", dWorldStep(_worldId, dttmp));
	            break;
	        }
	    } catch ( ... ) {
	        std::cout << "ERROR";
	        Log::errorLog() << "******************** Caught exeption in step function!*******************" << std::endl;
	        RW_THROW("ODESimulator caught exception.");
	    }

	    // if the solution is bad then we need to reduce timestep
	    if(!badLCPSolution){
            // this is onlu done to check that the stepsize was not too big
            //TIMING("Collision: ", dSpaceCollide(_spaceId, this, &nearCallback) );
            bool inCollision = false;
            TIMING("Collision Resolution: ", inCollision = detectCollisionsRW(tmpState, true) );

            if(!inCollision){
                //std::cout << "THERE IS NO PENETRATION" << std::endl;
                break;
            } else {

            	if(i>5){
                    // if we allready tried reducing the timestep then set the inCollisionFlag
                    // and let the contact resolution use the cached contacts
                    _prevStepEndedInCollision = true;
                    break;
                }

            }
	    } else {
	        badLCPcount++;
	        if( i>5 ){
	            bool inCollision = false;
	            TIMING("Collision Resolution: ", inCollision = detectCollisionsRW(tmpState, true) );
	            if(!inCollision){
	                //std::cout << "THERE IS NO PENETRATION" << std::endl;
	                break;
	            }
	        }
	    }

	    if( i==MAX_TIME_ITERATIONS-1){

            // TODO: register the objects that are penetrating such that we don't check them each time
            restoreODEState();
            // print all information available:
            Log::debugLog() << "----------------------- TOO LARGE PENETRATIONS --------------------\n";
            Log::debugLog() << "-- objects : " << _collidingObjectsMsg << "\n";
            Log::debugLog() << "-- time    : " << _time << "\n";
            Log::debugLog() << "-- dt orig : " << dt << "\n";
            Log::debugLog() << "-- dt div  : " << dttmp << "\n";
            Log::debugLog() << "-- step divisions: " << i << "\n";
            Log::debugLog() << "-- bad lcp count : " << badLCPcount << "\n";
            Log::debugLog() << "-- Bodies state  : \n";
            BOOST_FOREACH(dBodyID body, _allbodies){
                dReal vec[4];
                ODEBody *data = (ODEBody*) dBodyGetData(body);
                if(data!=NULL){
                    Log::debugLog() << "--- Body: " << data->getRwBody()->getName();
                } else {
                    Log::debugLog() << "--- Body: NoRWBODY";
                }
                drealCopy( dBodyGetPosition(body), vec, 3);
                Log::debugLog() << "\n---- pos   : " << printArray(vec, 3);
                drealCopy( dBodyGetQuaternion(body), vec, 4);
                Log::debugLog() << "\n---- rot   : " << printArray(vec, 4);
                drealCopy( dBodyGetLinearVel  (body), vec, 3);
                Log::debugLog() << "\n---- linvel: " << printArray(vec, 3);
                drealCopy( dBodyGetAngularVel (body), vec, 3);
                Log::debugLog() << "\n---- angvel: " << printArray(vec, 3);
                drealCopy( dBodyGetForce  (body), vec, 3);
                Log::debugLog() << "\n---- force : " << printArray(vec, 3);
                drealCopy( dBodyGetTorque (body), vec, 3);
                Log::debugLog() << "\n---- torque: " << printArray(vec, 3);
                Log::debugLog() << "\n";
            }
            Log::debugLog() << "--\n-- contacts: \n";
            BOOST_FOREACH(ContactPoint p, _allcontactsTmp){
                Log::debugLog() << "-- pos: "<< p.p << "\n";
                Log::debugLog() << "-- normal: "<< p.n << "\n";
                Log::debugLog() << "-- depth: "<< p.penetration << "\n";
            }

            Log::debugLog() << "----------------------- TOO LARGE PENETRATIONS --------------------" << std::endl;
            RW_THROW("Too Large Penetrations!");
            break;
        }
        badLCPSolution = false;
        // max penetration was then we step back to the last configuration and we try again
		dttmp /= 2;
		restoreODEState();
		tmpState = state;
	}
    state = tmpState;
	_oldTime = _time;
	_time += dttmp;

	// if the solution is bad then throw an exception
	//if(badLCPSolution)
	//    RW_WARN("Possibly bad LCP Solution.");
	//std::cout << "dt:" << dttmp << " dt_p:" << lastDt << " time:"<< _time << std::endl;

    Simulator::UpdateInfo conStepInfo;
    conStepInfo.dt = dttmp;
    conStepInfo.dt_prev = lastDt;
    conStepInfo.time = _time;
    conStepInfo.rollback = false;

	RW_DEBUGS("------------- Device post update:");
	//std::cout << "Device post update:" << std::endl;
	BOOST_FOREACH(ODEDevice *dev, _odeDevices){
	    dev->postUpdate(state);
	}

	RW_DEBUGS("------------- Update robwork bodies:");
	//std::cout << "Update robwork bodies:" << std::endl;
    // now copy all state info into state/bodies (transform,vel,force)
    for(size_t i=0; i<_odeBodies.size(); i++){
        //std::cout << "POST Update: " << _odeBodies[i]->getFrame()->getName() << std::endl;
        _odeBodies[i]->postupdate(state);
    }

    RW_DEBUGS("------------- Sensor update :");
    //std::cout << "Sensor update :" << std::endl;
    // update all sensors with the values of the joints
    BOOST_FOREACH(ODETactileSensor *odesensor, _odeSensors){
        odesensor->update(conStepInfo, state);
    }
    RW_DEBUGS("- removing joint group");
    // Remove all temporary collision joints now that the world has been stepped
    dJointGroupEmpty(_contactGroupId);
    // and the joint feedbacks that where used is also destroyed
    _nextFeedbackIdx=0;

	RW_DEBUGS("------------- Update trimesh prediction:");
	BOOST_FOREACH(ODEUtil::TriGeomData *data, _triGeomDatas){
	    if(!data->isGeomTriMesh)
	        continue;
	    dGeomID geom = data->geomId;
	    //if( dGeomGetClass(geom) != dTriMeshClass )
	    //    continue;
	    if (!data->isPlaceable)
	    	continue;
	    const dReal* Pos = dGeomGetPosition(geom);
	    const dReal* Rot = dGeomGetRotation(geom);

	    // Fill in the (4x4) matrix.
        dReal* p_matrix = data->mBuff[data->mBuffIdx];

        //if( !isClose(p_matrix, Pos, Rot, 0.001) ){
			p_matrix[ 0]=Rot[0]; p_matrix[ 1]=Rot[1]; p_matrix[ 2]=Rot[ 2 ]; p_matrix[ 3]=0;
			p_matrix[ 4]=Rot[4]; p_matrix[ 5]=Rot[5]; p_matrix[ 6]=Rot[ 6 ]; p_matrix[ 7]=0;
			p_matrix[ 8]=Rot[8]; p_matrix[ 9]=Rot[9]; p_matrix[10]=Rot[10 ]; p_matrix[11]=0;
			p_matrix[12]=Pos[0]; p_matrix[13]=Pos[1]; p_matrix[14]=Pos[ 2 ]; p_matrix[15]=1;

			// Flip to other matrix.
			data->mBuffIdx = !data->mBuffIdx;
        //}

        dGeomTriMeshSetLastTransform( geom, data->mBuff[data->mBuffIdx]);

	}
	//std::cout << "e";
	RW_DEBUGS("----------------------- END STEP --------------------------------");
	//std::cout << "-------------------------- END STEP --------------------------------" << std::endl;
}

void ODESimulator::addEmulatedContact(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& force, const rw::math::Vector3D<>& normal, dynamics::Body* b){
    std::vector<ODETactileSensor*> odesensors = getODESensors(getODEBodyId(b));
    BOOST_FOREACH(ODETactileSensor* sensor, odesensors){
        sensor->addContact(pos, force , normal, b);
    }
    ContactPoint point;
    point.p = pos;
    point.n = normal;
    point.nForce = dot(force, normal);
    _allcontacts.push_back(point);
}

rwsim::drawable::SimulatorDebugRender::Ptr ODESimulator::createDebugRender(){
    return _render;
}
// worlddimension -
// gravity

void ODESimulator::readProperties(){
    /*
    GLOBAL OPTIONS
    stepmethod - (WorldStep), QuickStep, FastStep1
    maxiterations - int (20)
    spacetype - simple, hash, (quad)
    quadtree depth - int
    WorldCFM - double [0-1]
    WorldERP - double [0-1]
    ContactClusteringAlg - Simple, Cube, ConvexHull, ConvexHullApprox


    ContactSurfaceLayer in meter default is 0.0001
    MaxSepDistance in meter default is 0.0005
    MaxPenetration in meter default is 0.00045
    MaxCorrectingVelocity in m/s default is 0.1

    PER BODY OPTIONS

    PER material-pair OPTIONS
        friction - coloumb,
        restitution -

     */
    _contactSurfaceLayer = _propertyMap.get<double>("ContactSurfaceLayer", 0.0001);
    _maxSepDistance = _propertyMap.get<double>("MaxSepDistance", 0.0005);
    _maxAllowedPenetration = _propertyMap.get<double>("MaxPenetration", _maxSepDistance);
	_contactMaxCorrectingVel = _propertyMap.get<double>("MaxCorrectingVelocity", 0.1);

	_maxIter = _propertyMap.get<int>("MaxIterations", 20);
	//std::string spaceTypeStr = _propertyMap.get<std::string>("SpaceType", "QuadTree");
	std::string spaceTypeStr = _propertyMap.get<std::string>("SpaceType", "Simple");
	//std::string stepStr = _propertyMap.get<std::string>("StepMethod", "WorldQuickStep");
	std::string stepStr = _propertyMap.get<std::string>("StepMethod", "WorldStep");
	if( stepStr=="WorldQuickStep" ){
		_stepMethod = WorldQuickStep;
	} else if( stepStr=="WorldStep" ) {
		_stepMethod = WorldStep;
	} else if( stepStr=="WorldFast1" ){
		_stepMethod = WorldFast1;
	} else {
		RW_THROW("ODE simulator property: Unknown step method!");
	}

	_worldCFM = _propertyMap.get<double>("WorldCFM", 0.0000001);
	_worldERP = _propertyMap.get<double>("WorldERP", 0.2);
	_clusteringAlgStr =  _propertyMap.get<std::string>("ContactClusteringAlg", "Box");

	if( spaceTypeStr == "QuadTree" )
		_spaceType = QuadTree;
	else if(spaceTypeStr == "Simple" )
		_spaceType = Simple;
	else if(spaceTypeStr == "HashTable" )
		_spaceType = HashTable;
	else
		_spaceType = QuadTree;
}

void ODESimulator::emitPropertyChanged(){
	readProperties();
}

static bool isODEInitialized = false;


namespace {

    void printMessage (int num, const char *msg1, const char *msg2, va_list ap)
    {
      fflush (stderr);
      fflush (stdout);
      if (num) fprintf (stderr,"\n%s %d: ",msg1,num);
      else fprintf (stderr,"\n%s: ",msg1);
      vfprintf (stderr,msg2,ap);
      fprintf (stderr,"\n");
      fflush (stderr);
    }

	void EmptyMessageFunction(int errnum, const char *msg, va_list ap){
		if(errnum==3){
		    // the LCP solution is bad. Try reducing the timestep
		    badLCPSolution = true;
		} else {
		    printMessage (errnum,"ODE Message",msg,ap);
		    //RW_WARN("ODE internal msg: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"");
		}
	}

	void ErrorMessageFunction(int errnum, const char *msg, va_list ap){
	    printMessage (errnum,"ODE Error",msg,ap);
		isInErrorGlobal=true;
		//RW_THROW("ODE internal Error: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"");
		RW_THROW("ODE ERROR");
	}

	void DebugMessageFunction(int errnum, const char *msg, va_list ap){
	    printMessage (errnum,"ODE INTERNAL ERROR",msg,ap);
		isInErrorGlobal=true;
		//dWorldCleanupWorkingMemory();
		RW_THROW("ODE INTERNAL ERROR");
	}




}


void ODESimulator::initPhysics(rw::kinematics::State& state)
{
    _propertyMap = _dwc->getEngineSettings();
    //CollisionSetup cSetup = Proximity::getCollisionSetup( *_dwc->getWorkcell() );

    //FramePairList excludeList = BasicFilterStrategy::getExcludePairList(*_dwc->getWorkcell(), cSetup);
    //BOOST_FOREACH(rw::kinematics::FramePair& pair, excludeList){
    //    _excludeMap[rw::kinematics::FramePair(pair.first,pair.second)] = 1;
    //}

    _bpstrategy = ownedPtr( new BasicFilterStrategy( _dwc->getWorkcell() ) );
    // build the frame map


    //std::vector<Object::Ptr> objects = _dwc->getWorkcell()->getObjects();
    BOOST_FOREACH(Body::Ptr body , _dwc->getBodies()){
        Object::Ptr obj = body->getObject();
        //std::cout << "objects: " << obj->getName() << std::endl;
        _narrowStrategy->addModel(obj);
    }

    std::vector<Frame*> frames = Kinematics::findAllFrames(_dwc->getWorkcell()->getWorldFrame(), state);
    BOOST_FOREACH(Frame *frame, frames){
        _frameToModels[*frame] = _narrowStrategy->getModel(frame);
    }


    readProperties();

    if(!isODEInitialized){
        dInitODE2(0);
        isODEInitialized = true;
        dSetErrorHandler(ErrorMessageFunction);
        dSetDebugHandler(DebugMessageFunction);
        dSetMessageHandler(EmptyMessageFunction);
    }


	// Create the world

	_worldId = dWorldCreate();

	// Create the space for geometric collision geometries
	WorkCellDimension wcdim = _dwc->getWorldDimension();
	switch(_spaceType){
        case(Simple):{
            _spaceId = dSimpleSpaceCreate(0);
        }
        break;
        case(HashTable):{
            _spaceId = dHashSpaceCreate(0);
        }
        break;
        case(QuadTree):{
            dVector3 center, extends;
            ODEUtil::toODEVector(wcdim.center, center);
            ODEUtil::toODEVector(wcdim.boxDim, extends);
            _spaceId = dQuadTreeSpaceCreate (0, center, extends, 7);
        }
        break;
        default:{
            RW_THROW("UNSUPPORTED SPACE TYPE!");
            break;
        }
	}

	// Create joint group
    _contactGroupId = dJointGroupCreate(0);

	// add gravity
    Vector3D<> gravity = _dwc->getGravity();
	dWorldSetGravity ( _worldId, gravity(0), gravity(1), gravity(2) );
	dWorldSetCFM ( _worldId, _worldCFM );
	dWorldSetERP ( _worldId, _worldERP );

	dWorldSetContactSurfaceLayer(_worldId, _contactSurfaceLayer);
	dWorldSetContactMaxCorrectingVel(_worldId, _contactMaxCorrectingVel);
	//dWorldSetAngularDamping()
    State initState = state;
    // first set the initial state of all devices.
    BOOST_FOREACH(DynamicDevice::Ptr device, _dwc->getDynamicDevices() ){
        JointDevice *jdev = dynamic_cast<JointDevice*>( &(device->getModel()) );
        if(jdev==NULL)
            continue;
        Q offsets = Q::zero( jdev->getQ(state).size() );
        jdev->setQ( offsets , initState );
    }

    //dCreatePlane(_spaceId,0,0,1,0);

    RW_DEBUGS( "- ADDING BODIES " );
    BOOST_FOREACH(Body::Ptr body, _dwc->getBodies() ){
        // check if a body is part
        if(!_dwc->inDevice(body) )
            addBody(body, state);
    }

	Frame *wframe = _dwc->getWorkcell()->getWorldFrame();
	_rwODEBodyToFrame[0] = wframe;

	RW_DEBUGS( "- ADDING DEVICES " );
	// this needs to be done in the correct order, which means if any device is attached to another then the
	// bottom one should  be added first.
    /*BOOST_FOREACH(DynamicDevice* device, _dwc->getDynamicDevices() ){
        JointDevice *jdev = dynamic_cast<JointDevice*>( &(device->getModel()) );
        if(jdev!=NULL){
            Q offsets = Q::zero( jdev->getQ(initState).size() );
            jdev->setQ( offsets , initState );
        }
    }*/
    BOOST_FOREACH(DynamicDevice::Ptr device, _dwc->getDynamicDevices() ){
        addDevice(device, initState);
    }

	 RW_DEBUGS( "- ADDING CONSTRAINTS " );
	 BOOST_FOREACH(Constraint::Ptr constraint, _dwc->getConstraints()) {
		 addConstraint(constraint);
	 }

    RW_DEBUGS( "- ADDING SENSORS " );
    BOOST_FOREACH(rwlibs::simulation::SimulatedSensor::Ptr sensor, _dwc->getSensors()){
    	addSensor(sensor, state);
	}

    RW_DEBUGS( "- ADDING CONTROLLERS " );
    BOOST_FOREACH(rwlibs::simulation::SimulatedController::Ptr controller, _dwc->getControllers()){
    	addController(controller);
	}

    RW_DEBUGS( "- CREATING MATERIAL MAP " );
    _odeMaterialMap = new ODEMaterialMap(_materialMap, _contactMap, _odeBodies);

	state.upgrade();
    RW_DEBUGS( "- RESETTING SCENE " );
	resetScene(state);
}

void ODESimulator::addController(rwlibs::simulation::SimulatedController::Ptr controller){
	if(!controller->isRegistered())
		controller->registerIn( _dwc->getWorkcell()->getStateStructure() );
	_controllers.push_back(controller);
}

void ODESimulator::removeController(rwlibs::simulation::SimulatedController::Ptr controller){

}


/*
ODEBody* ODESimulator::createBody(dynamics::Body* body, const rw::kinematics::State& state, dSpaceID spaceid)
{
    ODEBody *odeBody = NULL;
    if( RigidBody *rbody = dynamic_cast<RigidBody*>( body ) ){
        odeBody = createRigidBody(rbody, state, spaceid);
        dBodyID bodyId = odeBody->getBodyID();
        _bodies.push_back(bodyId);
        dBodySetAutoDisableFlag(bodyId, 0);
    } else if( KinematicBody *kbody = dynamic_cast<KinematicBody*>( body ) ) {
        odeBody = createKinematicBody(kbody, state, spaceid);
    } else if( FixedBody *fbody = dynamic_cast<FixedBody*>( body ) ) {
        odeBody = createFixedBody(fbody, state, spaceid);
    } else {
        RW_WARN("Unsupported body type, name: " << body->getName() );
    }
    return odeBody;
}
*/

void ODESimulator::addODEJoint(dJointID joint){
    _alljoints.push_back(joint);
}


void ODESimulator::addODEBody(dBodyID body){
    RW_DEBUGS("addODEBody: ");
    _allbodies.push_back(body);
}


void ODESimulator::addODEBody(ODEBody* odebody){
    RW_DEBUGS("Add ode body: " << odebody->getFrame()->getName() );
    if( _rwFrameToODEBody.find( odebody->getFrame() )!=_rwFrameToODEBody.end() )
        RW_THROW("Body with name \"" << odebody->getFrame()->getName() << "\" allready exists in the simulator! " );

   if(odebody->getRwBody()!=NULL){
       BOOST_FOREACH(rw::kinematics::Frame *f, odebody->getRwBody()->getFrames()){
           _rwFrameToODEBody[f] = odebody;
       }
   } else {
       _rwFrameToODEBody[odebody->getFrame()] = odebody;
   }
   BOOST_FOREACH(ODEUtil::TriGeomData* tgeom , odebody->getTriGeomData()){
       _frameToOdeGeoms[odebody->getFrame()] = tgeom->geomId;
   }
   _odeBodies.push_back(odebody);
   if(odebody->getType()!=ODEBody::FIXED)
       _allbodies.push_back(odebody->getBodyID());
}

void ODESimulator::addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State& state){
    RW_DEBUGS( "Add body: " << body->getName() );

    if( _rwFrameToODEBody.find( body->getBodyFrame() )!=_rwFrameToODEBody.end() )
        RW_THROW("Body with name \"" << body->getName() << "\" allready exists in the simulator! " );

    ODEBody *odeBody = NULL;
    bool odeBodyAdded = false;
    if( RigidBody *rbody = dynamic_cast<RigidBody*>( body.get() ) ){
        odeBody = ODEBody::makeRigidBody(rbody, _spaceId, this);
        //_allbodies.push_back(odeBody->getBodyID());
        dBodySetAutoDisableFlag(odeBody->getBodyID(), 0);
        odeBodyAdded = true;
    } else if( KinematicBody *kbody = dynamic_cast<KinematicBody*>( body.get() ) ) {
        odeBody = ODEBody::makeKinematicBody(kbody, _spaceId, this);
        //_allbodies.push_back(odeBody->getBodyID());
    } else if( FixedBody *fbody = dynamic_cast<FixedBody*>( body.get() ) ) {
        odeBody = ODEBody::makeFixedBody(fbody, _spaceId, this);
    } else {
        RW_WARN("Unsupported body type, name: " << body->getName() );
        return;
    }

    if (!odeBodyAdded) {
    	BOOST_FOREACH(Frame* f, odeBody->getRwBody()->getFrames() ){
    		_rwFrameToODEBody[f] = odeBody;
    	}
    	BOOST_FOREACH(ODEUtil::TriGeomData* tgeom , odeBody->getTriGeomData()){
    		_frameToOdeGeoms[odeBody->getFrame()] = tgeom->geomId;
    	}

    	_odeBodies.push_back(odeBody);
    }
	odeBody->setTransform(state);
}

void ODESimulator::addConstraint(Constraint::Ptr constraint) {
	const Body* const body1 = constraint->getBody1();
	const Body* const body2 = constraint->getBody2();
	const ODEBody* const ode1 = _rwFrameToODEBody[body1->getBodyFrame()];
	const ODEBody* const ode2 = _rwFrameToODEBody[body2->getBodyFrame()];
	if (ode1 == NULL)
		RW_THROW("Invalid body1 of Constraint: " << constraint->getName());
	if (ode2 == NULL)
		RW_THROW("Invalid body2 of Constraint: " << constraint->getName());
	ODEConstraint *odeConstraint = new ODEConstraint(constraint, ode1, ode2, this);
	_constraintToODEConstraint[constraint] = odeConstraint;
	_odeConstraints.push_back(odeConstraint);
}

void ODESimulator::addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State& nstate){

    //Frame *wframe = _dwc->getWorkcell()->getWorldFrame();
    rwsim::dynamics::DynamicDevice *device = dev.get();
    State state = nstate;

    // in case the base of one device is fixed onto another device
    // we need to reference the actual base frame with the body base frame
    // TODO: this feature should be replaced by composite body feature...
    if(dev->getBase()->getBodyFrame() != device->getModel().getBase() ){
        _rwFrameToODEBody[ device->getModel().getBase() ] = _rwFrameToODEBody[ dev->getBase()->getBodyFrame() ];
    }

    ODEBody* base = NULL;
    if( _rwFrameToODEBody.find( dev->getBase()->getBodyFrame() )==_rwFrameToODEBody.end() ){
        RW_DEBUGS("Creating base of device " << device->getKinematicModel()->getName());
        // the base has not yet been created, do it here
        Body::Ptr rwbase = device->getBase();
        addBody(rwbase, state);
        //_rwFrameToODEBody[ device->getModel().getBase() ] =
    }

    base = _rwFrameToODEBody[ device->getModel().getBase() ];
    if(base==NULL)
        RW_THROW("Invalid base of Device: " << device->getModel().getName());

    JointDevice *jdev = dynamic_cast<JointDevice*>( &(device->getModel()) );
    if(jdev!=NULL){
        Q offsets = Q::zero( jdev->getQ(state).size() );
        jdev->setQ( offsets , state );
    }

    if( RigidDevice* rdev = dynamic_cast<RigidDevice*>( device ) ){
        RW_DEBUGS("RigidDevice");
        ODEVelocityDevice* vdev = new ODEVelocityDevice(base, rdev, state, this);
        _odeDevices.push_back(vdev);
    } else  if( KinematicDevice* kdev = dynamic_cast<KinematicDevice*>( device ) ){
        RW_DEBUGS("KinematicDevice");
        ODEKinematicDevice *odekdev = new ODEKinematicDevice( kdev, state, this);
        _odeDevices.push_back(odekdev);
    } else if( SuctionCup* scup = dynamic_cast<SuctionCup*>( device ) ) {
        // make ODE suction cup simulation
        ODESuctionCupDevice *scup_ode = new ODESuctionCupDevice(base, scup , this, state);
        _odeDevices.push_back(scup_ode);
    } else {
        RW_WARN("Controller not supported!");
    }

}
#ifdef xdssdf
void ODESimulator::addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State& nstate){
    Frame *wframe = _dwc->getWorkcell()->getWorldFrame();
    rwsim::dynamics::DynamicDevice *device = dev.get();
    State state = nstate;

    JointDevice *jdev = dynamic_cast<JointDevice*>( &(device->getModel()) );
    if(jdev!=NULL){
        Q offsets = Q::zero( jdev->getQ(state).size() );
        jdev->setQ( offsets , state );
    }

    if( dynamic_cast<RigidDevice*>( device ) ){
        RW_DEBUGS("RigidDevice")
         // we use hashspace here because devices typically have
         // relatively few bodies
         dSpaceID space = dHashSpaceCreate( _spaceId );

         // add kinematic constraints from base to joint1, joint1 to joint2 and so forth
         RigidDevice *fDev = dynamic_cast<RigidDevice*>( device );
         Body *baseBody = fDev->getBase();
         Frame *base = baseBody->getBodyFrame();

         // Check if the parent of base has been added to the ODE world,
         // if not create a fixed body whereto the base can be attached
         Frame *baseParent = base->getParent();
         dBodyID baseParentBodyID = 0;
         if(_rwFrameToODEBody.find(baseParent) == _rwFrameToODEBody.end()){
             //RW_WARN("No parent data available, connecting base to world!");
             dBodyID baseParentBodyId = dBodyCreate(_worldId);
             ODEUtil::setODEBodyT3D(baseParentBodyId, Kinematics::worldTframe(baseParent,state));
             baseParentBodyID = 0;
             //_rwFrameToODEBody[baseParent] = baseParentBodyId;
             _rwFrameToODEBody[baseParent] = 0;
         } else {
             baseParentBodyID = _rwFrameToODEBody[baseParent]->getBodyID();
         }

         // now create the base
         RW_DEBUGS( "Create base");

         ODEBody *baseODEBody = createBody(baseBody, state, space);
         /*
         if( dynamic_cast<FixedBody*>(baseBody) ){
             RW_DEBUGS("- Fixed");
             baseODEBody = createFixedBody(baseBody, state, space);
         } else if(KinematicBody *kbody = dynamic_cast<KinematicBody*>(baseBody)){
             RW_DEBUGS("- Kinematic");
             baseODEBody = createKinematicBody(kbody, state, space);
         } else if(dynamic_cast<RigidBody*>(baseBody)){
             RW_DEBUGS("- Rigid");
             baseODEBody = createRigidBody(baseBody, state, space);
         } else {
             RW_THROW("Unknown body type of robot \""<< device->getModel().getName()<<"\" base");
         }
         */
         dBodyID baseBodyID = baseODEBody->getBodyID();
         _rwFrameToODEBody[ base ] = baseODEBody;

         // and connect the base to the parent using a fixed joint if the base is rigid
         // we only do this if the parent is another body, NOT if its the world
         if( dynamic_cast<RigidBody*>(baseBody) ){
        	 if(_rwFrameToODEBody[baseParent]!=0){
                 dJointID baseJoint = dJointCreateFixed(_worldId, 0);
                 dJointAttach(baseJoint, baseBodyID, baseParentBodyID);
                 dJointSetFixed(baseJoint);
                 std::string name = _rwFrameToODEBody[baseParent]->getRwBody()->getBodyFrame()->getName();
                 RW_DEBUGS("BASE: connecting base to body: "<<name);
        	 } else  {
                 RW_DEBUGS("BASE: connecting base to world");

        	 }
         }

         std::vector<ODEJoint*> odeJoints;
         Q maxForce = fDev->getForceLimit();
         RW_DEBUGS("BASE:" << base->getName() << "<--" << base->getParent()->getName() );

         size_t i =0;
         BOOST_FOREACH(RigidJoint *rjoint, fDev->getLinks() ){
             Joint *joint = rjoint->getJoint();
             Frame *parent = joint->getParent(state);
             RW_DEBUGS(parent->getName() << "<--" << joint->getName());

             ODEBody *odeParent;// = _rwFrameToODEBody[parent];
             //Frame *parentFrame = NULL;
             if(_rwFrameToODEBody.find(parent) == _rwFrameToODEBody.end() ){
                 // could be that the reference is
                 RW_WARN("odeParent is NULL, " << joint->getName() << "-->"
                         << parent->getName()
                         << " Using WORLD as parent");

                 odeParent = _rwFrameToODEBody[wframe];
                 _rwFrameToODEBody[parent] = odeParent;
             }
             odeParent = _rwFrameToODEBody[parent];
             //parentFrame = _rwODEBodyToFrame[odeParent];

             ODEBody* odeChild = createRigidBody(rjoint, state, space ); //_rwFrameToBtBody[joint];
             if(odeChild==NULL){
                 RW_WARN("odeChild is NULL, " << joint->getName());
                 RW_ASSERT(0);
             }

             Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
             Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
             Vector3D<> hpos = wTchild.P();
             //Transform3D<> wTparent = Kinematics::WorldTframe(parentFrame,initState);

             std::pair<Q, Q> posBounds = joint->getBounds();

              if(RevoluteJoint *rwjoint = dynamic_cast<RevoluteJoint*>(joint)){
                  RW_DEBUGS("Revolute joint");
                  const double qinit = rwjoint->getData(state)[0];
                  dJointID hinge = dJointCreateHinge (_worldId, 0);
                  dJointAttach(hinge, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                  dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));
                  dJointSetHingeParam(hinge, dParamCFM, 0.001);
                  // set the position limits
                  // TODO: these stops can only handle in interval [-Pi, Pi]
                  dJointSetHingeParam(hinge, dParamLoStop, posBounds.first[0] );
                  dJointSetHingeParam(hinge, dParamHiStop, posBounds.second[0] );

                  dJointID motor = NULL;
                  motor = dJointCreateAMotor (_worldId, 0);
                  dJointAttach(motor, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetAMotorNumAxes(motor, 1);
                  dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                  dJointSetAMotorAngle(motor,0, qinit);
                  dJointSetAMotorParam(motor,dParamFMax, maxForce(i) );
                  dJointSetAMotorParam(motor,dParamVel,0);
                  //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                  //dJointSetAMotorParam(Amotor,dParamHiStop,0);


                  // we use motor to simulate friction
                  /*
                  dJointID motor2 = dJointCreateAMotor (_worldId, 0);
                  dJointAttach(motor2, odeChild, odeParent);
                  dJointSetAMotorNumAxes(motor2, 1);
                  dJointSetAMotorAxis(motor2, 0, 1, haxis(0) , haxis(1), haxis(2));
                  dJointSetAMotorAngle(motor2,0, qinit);
                  dJointSetAMotorParam(motor2,dParamFMax, maxForce(i)/50 );
                  dJointSetAMotorParam(motor2,dParamVel,0);
                  */
                  ODEJoint *odeJoint = new ODEJoint(ODEJoint::Revolute, hinge, motor, odeChild->getBodyID(), rjoint);
                  _jointToODEJoint[rwjoint] = odeJoint;
                  odeJoints.push_back(odeJoint);
                  _allODEJoints.push_back(odeJoint);
              } else if( dynamic_cast<DependentRevoluteJoint*>(joint)){
                  RW_DEBUGS("DependentRevolute");
                  DependentRevoluteJoint *rframe = dynamic_cast<DependentRevoluteJoint*>(joint);
                  Joint *owner = &rframe->getOwner();
                  const double qinit = owner->getData(state)[0]*rframe->getScale()+0;

                  dJointID hinge = dJointCreateHinge (_worldId, 0);
                  dJointAttach(hinge, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                  dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

                  dJointID motor = dJointCreateAMotor (_worldId, 0);
                  dJointAttach(motor, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetAMotorNumAxes(motor, 1);
                  dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                  dJointSetAMotorAngle(motor,0, qinit);
                  dJointSetAMotorParam(motor,dParamFMax, 20/*maxForce(i)*/ );
                  dJointSetAMotorParam(motor,dParamVel,0);

                  ODEJoint *odeOwner = _jointToODEJoint[owner];
                  ODEJoint *odeJoint = new ODEJoint( ODEJoint::Revolute, hinge, motor,  odeChild->getBodyID(),
                                                     odeOwner, rframe ,
                                                     rframe->getScale(), 0 , rjoint);
                  odeJoints.push_back(odeJoint);
                  //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                  //dJointSetAMotorParam(Amotor,dParamHiStop,0);
                  _allODEJoints.push_back(odeJoint);
              } else if( PrismaticJoint *pjoint = dynamic_cast<PrismaticJoint*>(joint) ){

                  // test if another joint is dependent on this joint


                  //const double qinit = pjoint->getData(state)[0];
                  dJointID slider = dJointCreateSlider (_worldId, 0);
                  dJointAttach(slider, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));
                  //dJointSetHingeAnchor(slider, hpos(0), hpos(1), hpos(2));
                  dJointSetSliderParam(slider, dParamLoStop, posBounds.first[0] );
                  dJointSetSliderParam(slider, dParamHiStop, posBounds.second[0] );

                  dJointID motor = dJointCreateLMotor (_worldId, 0);
                  dJointAttach(motor, odeChild->getBodyID(), odeParent->getBodyID());
                  dJointSetLMotorNumAxes(motor, 1);
                  dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                  //dJointSetLMotorAngle(motor,0, qinit);

                  dJointSetLMotorParam(motor,dParamFMax, maxForce(i) );
                  dJointSetLMotorParam(motor,dParamVel,0);

                  //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                  //dJointSetAMotorParam(Amotor,dParamHiStop,0);
                  ODEJoint *odeJoint = new ODEJoint(ODEJoint::Prismatic, slider, motor, odeChild->getBodyID(), rjoint);
                  _jointToODEJoint[pjoint] = odeJoint;
                  odeJoints.push_back(odeJoint);
                  _allODEJoints.push_back(odeJoint);

              } else if( dynamic_cast<DependentPrismaticJoint*>(joint) ) {
                  RW_DEBUGS("DependentPrismaticJoint");
                  DependentPrismaticJoint *pframe = dynamic_cast<DependentPrismaticJoint*>(joint);
                  Joint *owner = &pframe->getOwner();
                  //const double qinit = owner->getData(state)[0]*pframe->getScale()+0;

                  ODEBody* ownerBody = _rwFrameToODEBody[owner];

                  dJointID slider = dJointCreateSlider (_worldId, 0);
                  //dJointAttach(slider, odeChild, odeParent);
                  dJointAttach(slider, odeChild->getBodyID(), ownerBody->getBodyID());
                  dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));

                  dJointID motor = dJointCreateLMotor (_worldId, 0);
                  //dJointAttach(motor, odeChild, odeParent);
                  dJointAttach(motor, odeChild->getBodyID(), ownerBody->getBodyID());
                  dJointSetLMotorNumAxes(motor, 1);
                  dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));

                 // std::cout << "i:" << i << " mforce_len: " << maxForce.size() << std::endl;
                  // TODO: should take the maxforce value of the owner joint
                  dJointSetLMotorParam(motor,dParamFMax, maxForce(i) );
                  dJointSetLMotorParam(motor,dParamVel,0);

                  ODEJoint *odeOwner = _jointToODEJoint[owner];
                  ODEJoint *odeJoint = new ODEJoint( ODEJoint::Prismatic, slider, motor, odeChild->getBodyID(),
                                                     odeOwner, pframe,
                                                     pframe->getScale(), 0 , rjoint);
                  odeJoints.push_back(odeJoint);

                  _allODEJoints.push_back(odeJoint);
              } else {
                  RW_WARN("Joint type not supported!");
              }

              i++;

           }
         _odeDevices.push_back( new ODEVelocityDevice(fDev, odeJoints, maxForce) );


     } else  if( KinematicDevice* kdev = dynamic_cast<KinematicDevice*>( device ) ){
         RW_DEBUGS("KinematicDevice");
         ODEKinematicDevice *odekdev = new ODEKinematicDevice( kdev, _spaceId);
         _odeDevices.push_back(odekdev);
     } else if( SuctionCup* scup = dynamic_cast<SuctionCup*>( device ) ) {
         RW_WARN("Creating suction cup!");
         //ODESuctionCupDevice *scup_ode = ODESuctionCupDevice::makeSuctionCup( scup , this, state);

         //BodyContactSensor::Ptr sensor = ownedPtr(new BodyContactSensor("SuctionCupSensor", scup->getEndBody()->getBodyFrame() ));
         //addSensor( sensor );
         // make ODE suction cup simulation
         ODESuctionCupDevice *scup_ode = new ODESuctionCupDevice(scup , this, state);
         _odeDevices.push_back(scup_ode);
     } else {
         RW_WARN("Controller not supported!");
     }
}
#endif
void ODESimulator::addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state){
	// make sure that the sensor is registered in the state
	if(!sensor->isRegistered())
		sensor->registerIn(state);
	_sensors.push_back(sensor);

	SimulatedSensor *ssensor = sensor.get();
	if( dynamic_cast<SimulatedFTSensor*>(ssensor) ){
	    // this sensor only monitores the forces acting on a specific constraint.
	    SimulatedFTSensor* tsensor = dynamic_cast<SimulatedFTSensor*>(sensor.get());
        Frame *parentFrame = tsensor->getBody1()->getBodyFrame();
        Frame *sensorFrame = tsensor->getBody2()->getBodyFrame();



        //std::cout << "Adding SimulatedTactileSensor: " << sensor->getSensor()->getName() << std::endl;
        //std::cout << "Adding SimulatedTactileSensor Frame: " << sensor->getSensor()->getFrame()->getName() << std::endl;
        if( _rwFrameToODEBody.find(sensorFrame)== _rwFrameToODEBody.end()){
            RW_THROW("The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics!");
        }
        if( _rwFrameToODEBody.find(parentFrame)== _rwFrameToODEBody.end()){
            RW_THROW("The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics!");
        }

        ODETactileSensor *odesensor = new ODETactileSensor(tsensor);
        ODEBody* parentOdeBody = _rwFrameToODEBody[parentFrame];
        ODEBody* sensorOdeBody = _rwFrameToODEBody[sensorFrame];

        // TODO: this should be a list of sensors for each body/frame
        //if(_odeBodyToSensor.find(odeBody->getBodyID())!=_odeBodyToSensor.end()){
        //  RW_ASSERT(0);
        //}
        _rwsensorToOdesensor[sensor] = odesensor;



        // we don't add this to the map since its only used for contact joint sensing
        // JAR: HOWEVER, the joint feedback does not seem to get contact forces on the child body, which it should....
        //_odeBodyToSensor[odeBody->getBodyID()].push_back( odesensor );
        _odeBodyToSensor[sensorOdeBody->getBodyID()].push_back(odesensor);
        _odeSensors.push_back(odesensor);

        //attach(parentOdeBody->getRwBody(), sensorOdeBody->getRwBody());

        // we find any permanent constraints between sensor frame and other bodies that are not the parent frame
        const ODEBody* const nonstaticBody = (sensorOdeBody->getType() == ODEBody::FIXED) ? parentOdeBody : sensorOdeBody;
        int nrJoints = dBodyGetNumJoints(nonstaticBody->getBodyID());

        for(int i=0;i<nrJoints;i++){
            const dJointID joint = dBodyGetJoint(nonstaticBody->getBodyID(), i);
            const dBodyID cbody1 = dJointGetBody(joint,0);
            const dBodyID cbody2 = dJointGetBody(joint,1);

            if (cbody1 == parentOdeBody->getBodyID() || cbody2 == parentOdeBody->getBodyID()) {
            	if(cbody1==sensorOdeBody->getBodyID()){
            		dJointFeedback *feedback = &_sensorFeedbacksGlobal[_nextFeedbackGlobalIdx];
            		_nextFeedbackGlobalIdx++;
            		dJointSetFeedback( joint, feedback );
            		odesensor->addFeedbackGlobal(feedback, tsensor->getBody1(), tsensor->getBody2(), 0);
            	} else if( cbody2==sensorOdeBody->getBodyID() ) {
            		dJointFeedback *feedback = &_sensorFeedbacksGlobal[_nextFeedbackGlobalIdx];
            		_nextFeedbackGlobalIdx++;
            		dJointSetFeedback( joint, feedback );
            		odesensor->addFeedbackGlobal(feedback, tsensor->getBody1(), tsensor->getBody2(), 1);
            	}
            }
        }


        /*
        // we find any permanent constraints between the two bodies and add permanent feedbacks
        int nrJoints = dBodyGetNumJoints(parentOdeBody->getBodyID());

        for(int i=0;i<nrJoints;i++){
            dJointID joint = dBodyGetJoint(parentOdeBody->getBodyID(), i);

            dBodyID cbody1 = dJointGetBody(joint,0);
            dBodyID cbody2 = dJointGetBody(joint,1);

            if(cbody1==sensorOdeBody->getBodyID()){
                dJointFeedback *feedback = &_sensorFeedbacksGlobal[_nextFeedbackGlobalIdx];
                _nextFeedbackGlobalIdx++;
                dJointSetFeedback( joint, feedback );
                odesensor->addFeedbackGlobal(feedback, tsensor->getBody1(), 0);
            } else if( cbody2==sensorOdeBody->getBodyID() ) {
                dJointFeedback *feedback = &_sensorFeedbacksGlobal[_nextFeedbackGlobalIdx];
                _nextFeedbackGlobalIdx++;
                dJointSetFeedback( joint, feedback );
                odesensor->addFeedbackGlobal(feedback, tsensor->getBody1(), 1);
            } else {
                // the constraint is not shared between both bodies
                continue;
            }
        }
        */

	} else if( dynamic_cast<SimulatedTactileSensor*>(ssensor) ){
	    // this is a general tactile sensor, only contact joints will be monitored.
        SimulatedTactileSensor *tsensor = dynamic_cast<SimulatedTactileSensor*>(sensor.get());
        Frame *bframe = tsensor->getFrame();

        RW_ASSERT(bframe!=NULL);

        //std::cout << "Adding SimulatedTactileSensor: " << sensor->getSensor()->getName() << std::endl;
        //std::cout << "Adding SimulatedTactileSensor Frame: " << sensor->getSensor()->getFrame()->getName() << std::endl;
        if( _rwFrameToODEBody.find(bframe)== _rwFrameToODEBody.end()){
            RW_THROW("The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics!" << bframe->getName());
        }

        ODETactileSensor *odesensor = new ODETactileSensor(tsensor);
        ODEBody* odeBody = _rwFrameToODEBody[bframe];

        // TODO: this should be a list of sensors for each body/frame
        //if(_odeBodyToSensor.find(odeBody->getBodyID())!=_odeBodyToSensor.end()){
        //	RW_ASSERT(0);
        //}
        _rwsensorToOdesensor[sensor] = odesensor;
        _odeBodyToSensor[odeBody->getBodyID()].push_back( odesensor );

        _odeSensors.push_back(odesensor);
    } else {
        _rwsensorToOdesensor[sensor] = NULL;
    }
}

void ODESimulator::removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor)
{
    std::vector<rwlibs::simulation::SimulatedSensor::Ptr> newsensors;
    BOOST_FOREACH(SimulatedSensor::Ptr oldsen , _sensors){
        if(sensor != oldsen )
            newsensors.push_back(oldsen);
    }
    _sensors = newsensors;
    Frame *bframe = sensor->getFrame();

    // check if sensor is not part of dynamicworkcell
    if( _dwc->findSensor( sensor->getName() ) != sensor){
    	// then remove it from the state
    	sensor->unregister();
    }

    if( _rwFrameToODEBody.find(bframe)== _rwFrameToODEBody.end()){
        return;
    }

    ODEBody* odeBody = _rwFrameToODEBody[bframe];

    if(_rwsensorToOdesensor.find(sensor)!=_rwsensorToOdesensor.end()){
        ODETactileSensor *tsensor = _rwsensorToOdesensor[sensor];
        // the sensor has an equivalent ode sensor remove that too
        std::vector<ODETactileSensor*>& odesensors = _odeBodyToSensor[odeBody->getBodyID()];

        if(odesensors.size()==0)
            return;

        std::vector<ODETactileSensor*> nodesensors;
        for(size_t i=0;i<odesensors.size();i++){
            if(odesensors[i]!=tsensor)
                nodesensors.push_back(odesensors[i]);
        }

        _odeBodyToSensor[odeBody->getBodyID()] = nodesensors;

        std::vector<ODETactileSensor*> newOdeSensors;
        BOOST_FOREACH(ODETactileSensor* oldsen , _odeSensors){
            if(tsensor != oldsen )
                newOdeSensors.push_back(oldsen);
        }

        if(newOdeSensors.size()==0)
            _odeBodyToSensor.erase( _odeBodyToSensor.find(odeBody->getBodyID()) );

        _odeSensors = newOdeSensors;

        delete tsensor;
    }

};


using namespace rw::proximity;

void ODESimulator::detectCollisionsContactDetector(const State& state) {
    /*if(_logContactingBodies){
        //_contactingBodies.clear();
    }*/
    
    //_contactPointsTmp.clear();
    //_contactPoints.clear();
    _contactingBodiesTmp.clear();
    
	std::vector<rwsim::contacts::Contact> contacts = _detector->findContacts(state);
    size_t numc = contacts.size();
    /*BOOST_FOREACH(rwsim::contacts::Contact &c, contacts) {
    	std::cout << "ModelA: " << c.getModelA()->getName() << " ModelB: " << c.getModelB()->getName() << " FrameA: " << c.getFrameA()->getName() << " FrameB: " << c.getFrameB()->getName() << " " << c.getNormal() << std::endl;
    }*/
    if(_rwcontacts.size()<numc){
        _rwcontacts.resize(numc);
        _contacts.resize(numc);
        _allcontacts.resize(numc);
	    _nrOfCon = (int)numc;
    }
    int ni = 0;
	BOOST_FOREACH(rwsim::contacts::Contact &contact, contacts) {
		dContact &con = _contacts[ni];

        ODEBody *a_data = _rwFrameToODEBody[contact.getFrameA()];
        ODEBody *b_data = _rwFrameToODEBody[contact.getFrameB()];
        if(a_data==NULL || b_data==NULL){
        	std::cout << "ODE Bodies not found" << std::endl;
        	std::cout << "Frame A: " << contact.getFrameA()->getName() << std::endl;
        	std::cout << "Frame B: " << contact.getFrameB()->getName() << std::endl;
            continue;
        }

		Vector3D<> n = -contact.getNormal();
		Vector3D<> p = contact.getPointA() - n*contact.getDepth()/2.; // In global coordinates

		ODEUtil::toODEVector(n, con.geom.normal);
		ODEUtil::toODEVector(p, con.geom.pos);

		if (contact.getDepth() < -_maxSepDistance)
			continue;

		double penDepth = _maxAllowedPenetration + contact.getDepth();
		con.geom.depth = penDepth;

        if( _enabledMap[*a_data->getFrame()]==0 || _enabledMap[*b_data->getFrame()]==0 ) {
        	std::cout << "disabled" << std::endl;
            continue;
        }

        dGeomID a_geom = _frameToOdeGeoms[a_data->getFrame()];
        dGeomID b_geom = _frameToOdeGeoms[b_data->getFrame()];
        if(a_geom==NULL || b_geom==NULL){
        	std::cout << "Geoms not found" << std::endl;
            continue;
        }


		con.geom.g1 = a_geom;
		con.geom.g2 = b_geom;

		ContactPoint &point = _rwcontacts[ni];

        if(_logContactingBodies){
            _contactingBodiesTmp[std::make_pair( contact.getFrameA()->getName(), contact.getFrameB()->getName())] = true; //.push_back( point );
            //_contactPointsTmp.push_back(boost::make_tuple(contact.getFrameA()->getName(),
			//	contact.getFrameB()->getName(),	point));
        }

		point.n = normalize( ODEUtil::toVector3D(con.geom.normal) );
		point.p = ODEUtil::toVector3D(con.geom.pos);
		point.penetration = con.geom.depth;
		point.userdata = (void*) &(_contacts[ni]);

		_allcontacts.push_back(point);

		std::vector<ODETactileSensor*>& odeSensorb1 = _odeBodyToSensor[a_data->getBodyID()];
	    std::vector<ODETactileSensor*>& odeSensorb2 = _odeBodyToSensor[b_data->getBodyID()];

	    dContact conSettings;
	    _odeMaterialMap->setContactProperties(conSettings, a_data, b_data);
	    con.surface = conSettings.surface;

	    _maxPenetration = std::max(point.penetration, _maxPenetration);

	    std::vector<dJointFeedback*> feedbacks;
	    std::vector<dContactGeom> feedbackContacts;
	    bool enableFeedback = false;
	    if(odeSensorb1.size()>0 || odeSensorb2.size()>0){
	    	enableFeedback = true;
	    }

	    dJointID c = dJointCreateContact (_worldId,_contactGroupId,&con);
	    dJointAttach (c, a_data->getBodyID(), b_data->getBodyID() );
	    if( enableFeedback ){
	    	RW_ASSERT( ((size_t)_nextFeedbackIdx)<_sensorFeedbacks.size());
	    	dJointFeedback *feedback = &_sensorFeedbacks[_nextFeedbackIdx];
	    	_nextFeedbackIdx++;
	    	feedbacks.push_back(feedback);
	    	feedbackContacts.push_back(con.geom);
	    	dJointSetFeedback( c, feedback );
	    }


	    if(enableFeedback && odeSensorb1.size()>0){
	    	BOOST_FOREACH(ODETactileSensor* sen, odeSensorb1){
	    		sen->addFeedback(feedbacks, feedbackContacts, a_data->getRwBody(), b_data->getRwBody(), 0);
	    	}
	    }
	    if(enableFeedback && odeSensorb2.size()>0){
	    	BOOST_FOREACH(ODETactileSensor* sen, odeSensorb2){
	    		sen->addFeedback(feedbacks, feedbackContacts, b_data->getRwBody(), a_data->getRwBody(), 1);
	    	}
	    }

		ni++;
	}
    //if(_logContactingBodies){
        _contactingBodies = _contactingBodiesTmp;
        //_contactPoints = _contactPointsTmp;
    //}

}

bool ODESimulator::detectCollisionsRW(rw::kinematics::State& state, bool onlyTestPenetration){
    //
    //std::cout << "detectCollisionsRW" << onlyTestPenetration << std::endl;
    ProximityFilter::Ptr filter = _bpstrategy->update(state);
    FKTable fk(state);
    ProximityStrategyData data;
    
    if (!onlyTestPenetration){
		_contactingBodiesTmp.clear();
	}

    // next we query the BP filter for framepairs that are possibly in collision
    while( !filter->isEmpty() ){
        const FramePair& pair = filter->frontAndPop();
        RW_DEBUGS(pair.first->getName() << " -- " << pair.second->getName());

        // and lastly we use the dispatcher to find the strategy the
        // is required to compute the narrowphase collision
        const ProximityModel::Ptr &a = _frameToModels[*pair.first];
        const ProximityModel::Ptr &b = _frameToModels[*pair.second];
        if(a==NULL || b==NULL){
            //std::cout << "No rwmodels:" << pair.first->getName() << " -- " << pair.second->getName() << std::endl;
            //std::cout << "No rwmodels:" << std::endl;
            continue;
        }

        // now find the "body" frame belonging to the frames
        ODEBody *a_data = _rwFrameToODEBody[pair.first];
        ODEBody *b_data = _rwFrameToODEBody[pair.second];
        if(a_data==NULL || b_data==NULL){
            //std::cout << "No ode bodies:" << pair.first->getName() << " -- " << pair.second->getName() << std::endl;
            continue;
        }

        if( _enabledMap[*a_data->getFrame()]==0 || _enabledMap[*b_data->getFrame()]==0 )
            continue;

        dGeomID a_geom = _frameToOdeGeoms[a_data->getFrame()];
        dGeomID b_geom = _frameToOdeGeoms[b_data->getFrame()];
        if(a_geom==NULL || b_geom==NULL){
            //std::cout << "No ode geoms:" << pair.first->getName() << " -- " << pair.second->getName() << std::endl;
            continue;
        }

        if( a_geom == b_geom ){
            //std::cout << "Same geoms:" << pair.first->getName() << " -- " << pair.second->getName() << std::endl;
            continue;
        }

        if(a_data  == b_data)
            continue;

        Transform3D<> aT = a_data->getTransform();
        Transform3D<> bT = b_data->getTransform();
        if(a_data->getFrame()!=pair.first)
            aT = aT * Kinematics::frameTframe(a_data->getFrame(),pair.first, state);
        if(b_data->getFrame()!=pair.second)
            bT = bT * Kinematics::frameTframe(b_data->getFrame(),pair.second, state);


        MultiDistanceResult *res;

        // test if we have cuttable object and if we have knife

        //bool isknifeandcuttable = false;
        bool aIsKnife = pair.first->getPropertyMap().has("Knife");
        bool bIsKnife = pair.second->getPropertyMap().has("Knife");
        //Frame *knifeF, *cuttableF;
        ProximityModel::Ptr knife, cuttable;
        Transform3D<> knifeT3d, cuttableT3d;
        if(aIsKnife && pair.second->getPropertyMap().has("Cuttable")){
        	// test if object is cuttable
        	//isknifeandcuttable = true;
        	knife = a;
        	knifeT3d = aT;
        	cuttable = b;
        	cuttableT3d = bT;
        	//knifeF = pair.first;
        	//cuttableF = pair.second;
        }
        else if(bIsKnife && pair.first->getPropertyMap().has("Cuttable") ){
        	//isknifeandcuttable = true;
        	knife = b;
        	knifeT3d = bT;
        	cuttable = a;
        	cuttableT3d = aT;
        	//knifeF = pair.second;
        	//cuttableF = pair.first;
        }

        // first make standard collision detection, if in collision then compute all contacts from dist query
        RW_DEBUGS( pair.first->getName() << " <--> " << pair.second->getName());

        if(onlyTestPenetration){
        	//std::cout << "ONLY TEST COL" << std::endl;
            data.setCollisionQueryType(CollisionStrategy::FirstContact);
            bool collides = _narrowStrategy->inCollision(a, aT, b, bT, data);
            if(collides){
                std::stringstream sstr;
                sstr << "IN COLLISION!!!!" << pair.first->getName() << " -- " << pair.second->getName() << std::endl;
                _collidingObjectsMsg = sstr.str();
                
                return true;
            }
            continue;
        }

        // TODO: if the object is a soft object then we need to add more contacts
        bool softcontact = false;
        double softlayer = 0.0;
        if( softcontact ){
            // change MAX_SEP_DISTANCE
            softlayer = 0.001;
        }

        data.setCollisionQueryType(CollisionStrategy::AllContacts);
        res = &_narrowStrategy->distances(a, aT, b, bT, _maxSepDistance+softlayer, data);

        // create all contacts
        size_t numc = res->distances.size();
        if(_rwcontacts.size()<numc){
            _rwcontacts.resize(numc);
            _contacts.resize(numc);
        }
        int ni = 0;
        for(size_t i=0;i<numc;i++){

            dContact &con = _contacts[ni]; 
            Vector3D<> p1 = aT * res->p1s[i];
            Vector3D<> p2 = aT * res->p2s[i];
            Vector3D<> n, p;

            if(res->distances[i]<0.00000001){
            	//std::cout << " penetrating " << std::endl;
                // the contact is penetrating and we therefore need compute the
            	// contact normal differently
            	std::pair< Vector3D<>, Vector3D<> > normals = _narrowStrategy->getSurfaceNormals(*res, (int)i);
            	// the second is described in b's refframe so convert both to world and combine them
            	Vector3D<> a_normal = aT.R() * normals.first;
            	Vector3D<> b_normal = bT.R() * normals.second;

            	n = -normalize( a_normal - b_normal );
            	p = p1;
            } else {
                double len = (p2-p1).norm2();
                n = (p2-p1)/(-len);
                //std::cout << "n: " << n << "\n";
                p = n*(res->distances[i]/2) + p1;
            }

            ODEUtil::toODEVector(n, con.geom.normal);
            ODEUtil::toODEVector(p, con.geom.pos);

            if( softcontact ){
                // scale the distances to fit into MAX_SEP_DISTANCE
                //std::cout << res->distances[i] << ";" << res->distances[i]*_maxSepDistance/(_maxSepDistance+softlayer) << std::endl;
                res->distances[i] *= _maxSepDistance/(_maxSepDistance+softlayer);
                //res->distances[i] *= _maxSepDistance/_maxSepDistance;
            }
            //double penDepth = MAX_SEP_DISTANCE-(res->distances[i]+(MAX_SEP_DISTANCE-MAX_PENETRATION));
            double penDepth = _maxAllowedPenetration - res->distances[i];
            con.geom.depth = penDepth;
            con.geom.g1 = a_geom;
            con.geom.g2 = b_geom;
            //std::cout << "Collision: " << p << n << penDepth << " " << res->distances[i]<< std::endl;
            
            ContactPoint &point = _rwcontacts[ni];
            point.n = normalize( ODEUtil::toVector3D(con.geom.normal) );
			point.p = ODEUtil::toVector3D(con.geom.pos);
			point.penetration = con.geom.depth;
			point.userdata = (void*) &(_contacts[ni]);
            
            if(_logContactingBodies) {
				//std::cout << "adding contact" << std::endl;
				_contactingBodiesTmp[std::make_pair(pair.first->getName(), pair.second->getName())] = true;//.push_back(point);
				//_contactPointsTmp.push_back(boost::make_tuple(pair.first->getName(), pair.second->getName(),
				//	point));
			}

            // friction direction between the bodies ...
            // Not necesary to calculate, unless we need explicit control
            ni++;
        }
        numc = ni;

        //if (numc < 100) {
			addContacts((int)numc, a_data, b_data, pair.first, pair.second);
		//}
        res->clear();
        // update the contact normal using the manifolds
    }
    
    //if(_logContactingBodies) {
		_contactingBodies = _contactingBodiesTmp;
		//_contactPoints = _contactPointsTmp;
	//}

    if(onlyTestPenetration){
        return false;
    }
    return false;
}

void ODESimulator::attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2){
    std::cout << "attach " << b1->getName() << " -- " << b2->getName() << std::endl;
    ODEBody *ob1 = _rwFrameToODEBody[ b1->getBodyFrame() ];
    ODEBody *ob2 = _rwFrameToODEBody[ b2->getBodyFrame() ];

    if(ob1==NULL )
        RW_THROW("Body b1 is not part of the simulation! "<< b1->getName());
    if(ob2==NULL )
        RW_THROW("Body b2 is not part of the simulation! "<< b2->getName());

    if(_attachConstraints.find(std::make_pair(b1->getBodyFrame(),b2->getBodyFrame()))!=_attachConstraints.end()){
        RW_THROW("Joints are allready attached!");
    }
    if(_attachConstraints.find(std::make_pair(b2->getBodyFrame(),b1->getBodyFrame()))!=_attachConstraints.end()){
        RW_THROW("Joints are allready attached!");
    }

    dJointID fjoint = dJointCreateFixed(_worldId, 0 );
    dJointAttach(fjoint, ob1->getBodyID(), ob2->getBodyID());
    _attachConstraints[std::make_pair(b1->getBodyFrame(),b2->getBodyFrame())] = fjoint;
    dJointSetFixed(fjoint);
}

void ODESimulator::detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2){
    ODEBody *ob1 = _rwFrameToODEBody[ b1->getBodyFrame() ];
    ODEBody *ob2 = _rwFrameToODEBody[ b2->getBodyFrame() ];

    if(ob1==NULL )
        RW_THROW("Body b1 is not part of the simulation! "<< b1->getName());
    if(ob2==NULL )
        RW_THROW("Body b2 is not part of the simulation! "<< b2->getName());

    if(_attachConstraints.find(std::make_pair(b1->getBodyFrame(),b2->getBodyFrame()))!=_attachConstraints.end()){
        dJointID fjoint = _attachConstraints[std::make_pair(b1->getBodyFrame(),b2->getBodyFrame())];
        dJointDestroy(fjoint);
        _attachConstraints.erase(std::make_pair(b1->getBodyFrame(),b2->getBodyFrame()));
        return;
    }
    if(_attachConstraints.find(std::make_pair(b2->getBodyFrame(),b1->getBodyFrame()))!=_attachConstraints.end()){
        dJointID fjoint = _attachConstraints[std::make_pair(b2->getBodyFrame(),b1->getBodyFrame())];
        dJointDestroy(fjoint);
        _attachConstraints.erase(std::make_pair(b2->getBodyFrame(),b1->getBodyFrame()));
        return;
    }
    RW_THROW("There are no attachments between body b1 and body b2!");
}


void ODESimulator::handleCollisionBetween(dGeomID o1, dGeomID o2)
{
	RW_DEBUGS("********************handleCollisionBetween ************************** ")
    // Create an array of dContact objects to hold the contact joints
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    //std::cout << "Collision: " << b1 << " " << b2 << std::endl;
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)){
    	RW_DEBUGS(b1 <<"&&" << b2 <<"&&" << dAreConnectedExcluding (b1,b2,dJointTypeContact));
        return;
    }

    ODEBody *dataB1;
    if( b1==NULL ) {
        dataB1 = (ODEBody*) dGeomGetData(o1);
    } else {
        dataB1 = (ODEBody*) dBodyGetData(b1);
    }
    //RW_DEBUGS("- get data2")
    ODEBody *dataB2;
    if( b2==NULL ) {
        dataB2 = (ODEBody*) dGeomGetData(o2);
    } else {
        dataB2 = (ODEBody*) dBodyGetData(b2);
    }

    if(dataB1 == NULL || dataB2==NULL ){
        //if(dataB1!=NULL)
        //	std::cout << "b1: " << dataB1->getFrame()->getName() << std::endl;
        //if(dataB2!=NULL)
        //	std::cout << "b2: " << dataB2->getFrame()->getName() << std::endl;
    	return;
    }

    RW_DEBUGS("- get data3 " << dataB1 << " " << dataB2)
    Frame *frameB1 = dataB1->getFrame();
    Frame *frameB2 = dataB2->getFrame();

    if(frameB1 == frameB2)
        return;
    RW_DEBUGS("- check enabled Map")
    // if any of the bodies are disabled then discard them
    if( _enabledMap[*frameB1]==0 || _enabledMap[*frameB2]==0 )
        return;
    RW_DEBUGS("- check exclude Map")
    // check if the frames are in the exclude list, if they are then return
    rw::kinematics::FramePair pair(frameB1,frameB2);
    if( _excludeMap.has( pair ) )
        return;
    //std::cout << frameB1->getName() << " " << frameB2->getName() << std::endl;
    // update the
    //std::vector<ContactManifold> &manifolds = _manifolds[pair];
    //BOOST_FOREACH(ContactManifold &manifold, manifolds){
        //manifold.update(); // TODO: use transform between objects to update manifold
    //}
    RW_DEBUGS("- do collide")
    int numc;
    // do the actual collision check

    numc = dCollide(o1, o2,
                        ((int)_contacts.size()-1) , &_contacts[0].geom,
                        sizeof(dContact));

    if( numc >= (int)_contacts.size()-1 ){
        numc = (int)_contacts.size()-2;
        RW_WARN( "------- Too small collision buffer ------" );
    }

    addContacts(numc, dataB1, dataB2, frameB1, frameB2);
}

rw::math::Vector3D<> ODESimulator::addContacts(int numc, ODEBody* dataB1, ODEBody* dataB2, Frame *f1, Frame *f2){
    RW_DEBUGS("Add Contacts");
    if(numc==0){
        RW_DEBUGS("No collisions detected!");
        return Vector3D<>(0,0,0);
    }

    //RW_DEBUGS("- detected: " << frameB1->getName() << " " << frameB2->getName());

    // check if any of the bodies are actually sensors
    std::vector<ODETactileSensor*>& odeSensorb1 = _odeBodyToSensor[dataB1->getBodyID()];
    std::vector<ODETactileSensor*>& odeSensorb2 = _odeBodyToSensor[dataB2->getBodyID()];

    // perform contact clustering
    //double threshold = std::min(dataB1->getCRThres(), dataB2->getCRThres());
    //std::cout << "Numc: " << numc << std::endl;



    int j=0;
    for(int i=0;i<numc;i++){
        ContactPoint &point = _rwcontacts[j];
        const dContact &con = _contacts[i];

        //if(con.geom.depth>0.01)
        //    continue;

        point.n = normalize( ODEUtil::toVector3D(con.geom.normal) );
        point.p = ODEUtil::toVector3D(con.geom.pos);
        point.penetration = con.geom.depth;
        point.userdata = (void*) &(_contacts[i]);
        RW_DEBUGS("-- Depth/Pos  : " << con.geom.depth << " " << printArray(con.geom.normal,3));
        RW_DEBUGS("-- Depth/Pos p: " << point.penetration << " p:" << point.p << " n:"<< point.n);

        //_allcontacts.push_back(point);
        j++;
        //std::cout << "n: " << point.n << " p:" << point.p << " dist: "
		//		  << MetricUtil::dist2(point.p,_rwcontacts[std::max(0,i-1)].p) << std::endl;
    }
    numc = j;
    if((int)_srcIdx.size()<numc)
        _srcIdx.resize(numc);
    if((int)_dstIdx.size()<numc)
        _dstIdx.resize(numc);

    int fnumc = ContactCluster::normalThresClustering(
									&_rwcontacts[0], numc,
                                    &_srcIdx[0], &_dstIdx[0],
                                    &_rwClusteredContacts[0],
                                    10*Deg2Rad);

    //std::cout << "Threshold: " << threshold << " numc:" << numc << " fnumc:" << fnumc << std::endl;
    RW_DEBUGS(" numc:" << numc << " fnumc:" << fnumc);
    //RW_DEBUGS("Nr of average contact points in cluster: " << numc/((double)fnumc));

    // we create a manyfold per cluster, and only use the most significant points

    std::vector<ContactPoint> &src = _rwcontacts;
    std::vector<ContactPoint> &dst = _rwClusteredContacts;

    //std::cout << "Number of contact clusters: " << fnumc << std::endl;
    /*
    for(int i=0;i<fnumc;i++)
        std::cout << "i:" << i << " -> " << dst[i].penetration << " " << dst[i].p<< " " << dst[i].n << std::endl;

    std::cout << "Dst idx array: " << std::endl;
    for(int i=0;i<_dstIdx.size();i++)
        std::cout << "i:" << i << " -> " << _dstIdx[i] << " " << src[_dstIdx[i] ].p<< " " << src[_dstIdx[i] ].n << std::endl;

    std::cout << "Src idx array: " << std::endl;
    for(int i=0;i<_srcIdx.size();i++)
        std::cout << "i:" << i << " -> " << _srcIdx[i] << std::endl;
    */
    // test manifold functionality
    std::vector< OBRManifold > manifolds;
    // for each cluster we fit a manifold
    for(int i=0;i<fnumc;i++){
        int idxFrom = _srcIdx[i];
        const int idxTo = _srcIdx[i+1];
        // locate the manifold that idxFrom is located in
        OBRManifold manifold(15*Deg2Rad,0.2);

        //std::cout << "Adding clustered points to manifold!" << std::endl;

        for(;idxFrom<idxTo; idxFrom++){
            ContactPoint &point = src[_dstIdx[idxFrom]];
            //std::cout << point.p << std::endl;

            if( !manifold.addPoint(point) ){
                // hmm, create a new manifold for this point
                // TODO: Create a new manifold here to hold points that do not belong
                //std::cout << "POINT NOT IN MANIFOLD................................" << std::endl;
            }
        }
        manifolds.push_back(manifold);
    }
    RW_DEBUGS("Fibnd Contacts from manifolds");
    // run through all manifolds and get the contact points that will be used.
    int contactIdx = 0;
    rw::math::Vector3D<> contactNormalAvg(0,0,0);
    BOOST_FOREACH(OBRManifold& obr, manifolds){
        contactNormalAvg += obr.getNormal();
        int nrContacts = obr.getNrOfContacts();
        // if the manifold area is very small (area requires at least 3 points) then we only use a single point
        // for contact
        Vector3D<> hf = obr.getHalfLengths();
        if(hf(0)*hf(1)<(0.001*0.001) && nrContacts >= 3){
            _allcontacts.push_back( obr.getDeepestPoint() );
            RW_ASSERT( contactIdx<(int)dst.size() );
            dst[contactIdx] = obr.getDeepestPoint();
            contactIdx++;
        } else {
            //std::cout << "Manifold: " << nrContacts << ";" << std::endl;
            for(int j=0;j<nrContacts; j++){
                _allcontacts.push_back( obr.getContact(j) );
                RW_ASSERT( contactIdx<(int)dst.size() );
                dst[contactIdx] = obr.getContact(j);
                contactIdx++;
            }
        }
    }
    RW_DEBUGS("Add material map");
    // get the friction data and contact settings
    dContact conSettings;
    RW_ASSERT(_odeMaterialMap);
    _odeMaterialMap->setContactProperties(conSettings, dataB1, dataB2);

    // TODO: collisionMargin should also be specified per object
    //double collisionMargin = _dwc->getCollisionMargin();

    RW_DEBUGS("- Filtered contacts: " << numc << " --> " << fnumc);


    //std::cout << "- Filtered contacts: " << numc << " --> " << fnumc << std::endl;
    //Frame *world = _dwc->getWorkcell()->getWorldFrame();
    _nrOfCon = fnumc;
    std::vector<dJointFeedback*> feedbacks;
    std::vector<dContactGeom> feedbackContacts;
    bool enableFeedback = false;
    if(odeSensorb1.size()>0 || odeSensorb2.size()>0){
        //if( (dataB1->getType()!=ODEBody::FIXED) && (dataB2->getType()!=ODEBody::FIXED) ){
        //    if( (f1 != world) && (f2!=world) ){
                enableFeedback = true;
                //std::cout << "- detected: " << frameB1->getName() << " " << frameB2->getName() << std::endl;
        //    }
        //}
    }
    Vector3D<> cNormal(0,0,0);
    double maxPenetration = 0;
    // Run through all contacts and define contact constraints between them
    //std::vector<ContactPoint> &contactPointList = _rwClusteredContacts; int num = numc;
    std::vector<ContactPoint> &contactPointList = dst; int num = contactIdx;

    /*
    if( (dataB2->getFrame()->getName() == "SuctionGripper.Joint4" ||
            dataB1->getFrame()->getName() == "SuctionGripper.Joint4")
                    ){
        // adding suction force
        //con.geom.depth = -point->penetration;
        std::cout << "---------------------------------------------------" << std::endl;
        contactPointList[num] = contactPointList[0];
        contactPointList[num].n = -contactPointList[num].n;
        contactPointList[num].penetration = 0.0001;
        num++;
    }
    */

    // test if frame is conveyor
    Vector3D<> dir1; // in world coordinates
    bool hasConv = false;
    if( f1->getPropertyMap().has("Conveyor") ){
    	Q cdata = f1->getPropertyMap().get<Q>("Conveyor");
    	dir1[0] = cdata(0);
    	dir1[1] = cdata(1);
    	dir1[2] = cdata(2);
    	Transform3D<> wTf1 = Kinematics::worldTframe(f1,*_stepState);
    	dir1 = wTf1.R() * dir1;
    	hasConv = true;
    }

    if( f2->getPropertyMap().has("Conveyor") ){
    	Q cdata = f2->getPropertyMap().get<Q>("Conveyor");
    	dir1[0] = cdata(0);
    	dir1[1] = cdata(1);
    	dir1[2] = cdata(2);
    	Transform3D<> wTf2 = Kinematics::worldTframe(f2,*_stepState);
    	dir1 = wTf2.R() * dir1;
    	hasConv = true;
    }

    for (int i = 0; i < num; i++) {
        ContactPoint *point = &contactPointList[i];
        point->n = normalize(point->n);
        dContact &con = *((dContact*)point->userdata);

        //std::cout << point->penetration << " " << point->n << "  " << point->p << "  " << std::endl;
        cNormal += point->n;
        double rwnlength = MetricUtil::norm2(point->n);
        if((0.9>rwnlength) || (rwnlength>1.1)){
        	//std::cout <<  "\n\n Normal not normalized _0_ !\n"<<std::endl;
        	continue;
        }
        ODEUtil::toODEVector(point->n, con.geom.normal);
        ODEUtil::toODEVector(point->p, con.geom.pos);

        //double odenlength = sqrt( con.geom.normal[0]*con.geom.normal[0] +
        //                          con.geom.normal[1]*con.geom.normal[1] +
        //                          con.geom.normal[2]*con.geom.normal[2] );


        //if( (0.9>odenlength) || (odenlength>1.1) )
        //	std::cout <<  "\n\n Normal not normalized _1_ !\n"<<std::endl;

        con.geom.depth = point->penetration;

        maxPenetration = std::max(point->penetration, maxPenetration);
        _maxPenetration = std::max(point->penetration, _maxPenetration);

        //if( con.geom.depth <= 0){
        //    continue;
        //}

        RW_DEBUGS("-- Depth/Pos  : " << con.geom.depth << " " << printArray(con.geom.pos,3));
        //RW_DEBUGS("-- Depth/Pos p: " << point.penetration << " " << point.p );

        // currently we use the best possible ode approximation to Coloumb friction
        con.surface = conSettings.surface;

        if( hasConv ){
        	RW_DEBUGS("IS Conveour dir: " << dir1);

        	// change friction direction to point in direction of dir1
        	con.surface.mode |= dContactFDir1;
        	con.surface.mode |= dContactMotion1;
        	con.surface.motion1 = MetricUtil::norm2( dir1 );

        	// friction dir1 will point in direction of conveyor movement
        	Vector3D<> dir2 = cross(dir1, point->n);
        	Vector3D<> moddir1 = cross(point->n, dir2);
        	//double len = dot( normalize(moddir1), dir1 );
        	moddir1 = normalize( moddir1 );
            ODEUtil::toODEVector(moddir1, con.fdir1);
        	//std::cout << "--> " << moddir1 << std::endl;

        }
        //con.surface.motion1 =

        RW_DEBUGS("-- create contact ");
        dJointID c = dJointCreateContact (_worldId,_contactGroupId,&con);
        RW_DEBUGS("-- attach bodies");
        dJointAttach (c, dataB1->getBodyID(), dataB2->getBodyID() );
        RW_DEBUGS("--enable feedback");
        // We can only have one Joint feedback per joint so the sensors will have to share
        if( enableFeedback ){
            RW_ASSERT( ((size_t)_nextFeedbackIdx)<_sensorFeedbacks.size());
            dJointFeedback *feedback = &_sensorFeedbacks[_nextFeedbackIdx];
            _nextFeedbackIdx++;
            feedbacks.push_back(feedback);
            feedbackContacts.push_back(con.geom);
            dJointSetFeedback( c, feedback );
        }
    }
    //std::cout << "_maxPenetration: " << _maxPenetration << " meter" << std::endl;
    if(enableFeedback && odeSensorb1.size()>0){
    	//std::cout << "----------- ADD FEEDBACK\n";
        BOOST_FOREACH(ODETactileSensor* sen, odeSensorb1){
            sen->addFeedback(feedbacks, feedbackContacts, dataB2->getRwBody(), dataB1->getRwBody(), 0);
        }
        //odeSensorb1->setContacts(result,wTa,wTb);
    }
    if(enableFeedback && odeSensorb2.size()>0){
    	//std::cout << "----------- ADD FEEDBACK\n";
        BOOST_FOREACH(ODETactileSensor* sen, odeSensorb2){
    	        sen->addFeedback(feedbacks, feedbackContacts, dataB1->getRwBody(), dataB2->getRwBody(), 1);
        }
        //odeSensorb2->setContacts(result,wTa,wTb);
    }

    // if either body was a sensor then find all contacts within some small threshold
    /*
    if( enableFeedback ){
        MultiDistanceResult result;
        Transform3D<> wTa, wTb;
        if( !((dataB2->getType()==ODEBody::FIXED) || (dataB1->getType()==ODEBody::FIXED)) ){
            //std::cout << "MAX PENETRATION: " << maxPenetration << std::endl;
            Vector3D<> normal = normalize(cNormal);
            wTa = dataB1->getTransform();
            wTb = dataB2->getTransform();
            wTb.P() -= normal*(maxPenetration+0.001);
            bool res = _narrowStrategy->getDistances(result,
                                          frameB1,wTa,
                                          frameB2,wTb,
                                          0.002+maxPenetration);

            for(int i=0;i<result.distances.size();i++){
                ContactPoint cp;
                cp.n = wTa*result.p2s[i]-wTa*result.p1s[i];
                cp.p = wTa*result.p1s[i];
                _allcontacts.push_back(cp);
            }
        }
        if(odeSensorb1){
            odeSensorb1->addFeedback(feedbacks, feedbackContacts, 0);
            odeSensorb1->setContacts(result,wTa,wTb);
        }
        if(odeSensorb2){
            odeSensorb2->addFeedback(feedbacks, feedbackContacts, 1);
            odeSensorb2->setContacts(result,wTa,wTb);
        }
    }
    */

    //std::cout << "************ COL DONE *****************" << std::endl;

    return normalize( contactNormalAvg);
}

void ODESimulator::addContacts(std::vector<dContact>& contacts, size_t nr_con, ODEBody* dataB1, ODEBody* dataB2){

    RW_ASSERT(nr_con<=contacts.size());
    if(dataB1==NULL || dataB2==NULL){
        RW_DEBUGS("Bodies are NULL");
        RW_THROW("Bodies are NULL");
    }

    std::vector<ODETactileSensor*>& odeSensorb1 = _odeBodyToSensor[dataB1->getBodyID()];
    std::vector<ODETactileSensor*>& odeSensorb2 = _odeBodyToSensor[dataB2->getBodyID()];


    std::vector<dJointFeedback*> feedbacks;
    std::vector<dContactGeom> feedbackContacts;
    bool enableFeedback = false;
    if(odeSensorb1.size()>0 || odeSensorb2.size()>0){
        enableFeedback = true;
    }
    for (size_t i = 0; i < nr_con; i++) {
        dContact con = contacts[i];

        if( ODEUtil::toVector3D(con.geom.normal).norm2() >1.0001  ||
                ODEUtil::toVector3D(con.geom.normal).norm2() >0.9999 ){
            RW_WARN("Contact normals are bad! skipping contact.");
            continue;
        }

        // add to all contacts
        ContactPoint point;
        point.n = normalize( ODEUtil::toVector3D(con.geom.normal) );
        point.p = ODEUtil::toVector3D(con.geom.pos);
        point.penetration = con.geom.depth;
        _allcontacts.push_back(point);


        RW_DEBUGS("-- create contact ");
        dJointID c = dJointCreateContact (_worldId,_contactGroupId,&con);
        RW_DEBUGS("-- attach bodies");
        dJointAttach (c, dataB1->getBodyID(), dataB2->getBodyID() );
        RW_DEBUGS("--enable feedback");
        // We can only have one Joint feedback per joint so the sensors will have to share
        if( enableFeedback ){
            RW_ASSERT( ((size_t)_nextFeedbackIdx)<_sensorFeedbacks.size());
            dJointFeedback *feedback = &_sensorFeedbacks[_nextFeedbackIdx];
            _nextFeedbackIdx++;
            feedbacks.push_back(feedback);
            feedbackContacts.push_back(con.geom);
            dJointSetFeedback( c, feedback );
        }
    }

    //std::cout << "_maxPenetration: " << _maxPenetration << " meter" << std::endl;
    if(enableFeedback && odeSensorb1.size()>0){
        //std::cout << "----------- ADD FEEDBACK\n";
        BOOST_FOREACH(ODETactileSensor* sen, odeSensorb1){
            sen->addFeedback(feedbacks, feedbackContacts, dataB2->getRwBody(), dataB1->getRwBody(), 0);
        }
        //odeSensorb1->setContacts(result,wTa,wTb);
    }
    if(enableFeedback && odeSensorb2.size()>0){
        //std::cout << "----------- ADD FEEDBACK\n";
        BOOST_FOREACH(ODETactileSensor* sen, odeSensorb2){
                sen->addFeedback(feedbacks, feedbackContacts, dataB1->getRwBody(), dataB2->getRwBody(), 1);
        }
        //odeSensorb2->setContacts(result,wTa,wTb);
    }


}


void ODESimulator::resetScene(rw::kinematics::State& state)
{
    if(isInErrorGlobal){
        // delete world and reinitialize everything

    }

	isInErrorGlobal = false;
	_time = 0.0;

	// first run through all rigid bodies and set the velocity and force to zero
	RW_DEBUGS("- Resetting bodies: " << _allbodies.size());
	BOOST_FOREACH(dBodyID body, _allbodies){
	    dBodySetLinearVel  (body, 0, 0, 0);
	    dBodySetAngularVel (body, 0, 0, 0);
	    dBodySetForce  (body, 0, 0, 0);
	    dBodySetTorque (body, 0, 0, 0);
	}

	BOOST_FOREACH(ODEBody* body, _odeBodies){
	    body->reset(state);
	}

	// next the position need be reset to what is in state
	// run through all rw bodies and set the body position accordingly
	RW_DEBUGS("- Resetting sensors: " << _odeSensors.size());
	BOOST_FOREACH(ODETactileSensor* sensor, _odeSensors){
	    sensor->clear();
	}

	RW_DEBUGS("- Resetting devices: " << _odeDevices.size());
	// run through all devices and set the rigid bodies accoringly
	BOOST_FOREACH(ODEDevice *device, _odeDevices){
	    device->reset(state);
	}
	RW_DEBUGS("Finished reset!!");
}

void ODESimulator::disableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2){
    _bpstrategy->addRule( ProximitySetupRule::makeExclude(b1->getName(), b2->getName()) );
}


void ODESimulator::enableCollision(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2){
    _bpstrategy->addRule( ProximitySetupRule::makeInclude(b1->getName(), b2->getName()) );
}


void ODESimulator::exitPhysics()
{
	BOOST_FOREACH(ODEConstraint* constraint, _odeConstraints) {
		delete constraint;
	}
	_odeConstraints.clear();
	
	BOOST_FOREACH(ODEDevice* vdev, _odeDevices) {
		delete vdev;
	}
	_odeDevices.clear();

	BOOST_FOREACH(ODETactileSensor* sensor, _odeSensors) {
		delete sensor;
	}
	_odeSensors.clear();

	while (_odeBodies.size() > 0) {
		ODEBody* body = _odeBodies[0];
        if(!_dwc->inDevice(body->getRwBody()) )
    		delete body;
		std::vector<ODEBody*>::iterator it = _odeBodies.begin();
		while (it < _odeBodies.end()) {
			if (*it == body)
				it = _odeBodies.erase(it);
			else
				it++;
		}
	}
	_odeBodies.clear();

	// only if init physics have been called
	dJointGroupDestroy( _contactGroupId );
	dWorldDestroy(_worldId);
	dSpaceDestroy(_spaceId);

	delete _odeMaterialMap;

	_frameToModels.clear();
	_bpstrategy = NULL;

}
