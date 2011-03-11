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

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <rw/models/JointDevice.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/geometry/TriangleUtil.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>

#include <boost/foreach.hpp>

#include <rw/kinematics/FramePairMap.hpp>

#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Cylinder.hpp>

#include "ODEKinematicDevice.hpp"
#include "ODEVelocityDevice.hpp"
#include "ODEDebugRender.hpp"
#include "ODEUtil.hpp"

#include <rwsim/dynamics/OBRManifold.hpp>

#include <rw/proximity/Proximity.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>
#include <rw/common/Log.hpp>

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp> // !
#include <boost/lambda/construct.hpp>
#include <boost/function.hpp>

#include <fstream>
#include <iostream>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::sensor;

using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::models;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::common;

using namespace rwlibs::simulation;
using namespace rwlibs::proximitystrategies;

#define INITIAL_MAX_CONTACTS 1000

//#define RW_DEBUGS( str ) std::cout << str  << std::endl;
#define RW_DEBUGS( str )
/*
#define TIMING( str, func ) \
    { long start = rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }
*/
#define TIMING( str, func ) {func;}

namespace {

    // register the ODEPhysicsEngine with the factory
    struct InitStruct {
        InitStruct(){
            //std::cout << "************************** INITIALIZE ODE *********************" << std::endl;

            rwsim::simulator::PhysicsEngineFactory::makePhysicsEngineFunctor odephysics =
                    boost::lambda::bind( boost::lambda::new_ptr<rwsim::simulator::ODESimulator>(), boost::lambda::_1);

            rwsim::simulator::PhysicsEngineFactory::addPhysicsEngine("ODE DSfdsfiudif", odephysics);
        }
    };

    //InitStruct initializeStaticStuff;




   Vector3D<> toVector3D(const dReal *v){
        return Vector3D<>(v[0],v[1],v[2]);
    }


	void setODEBodyMass(dMass *m, double mass, Vector3D<> c, InertiaMatrix<> I){
		dReal i11 = I(0,0);
		dReal i22 = I(1,1);
		dReal i33 = I(2,2);
		dReal i12 = I(0,1);
		dReal i13 = I(0,2);
		dReal i23 = I(1,2);
		dMassSetParameters(m, mass,
                           c(0), c(1), c(2),
						   i11, i22, i33,
						   i12, i13, i23);
	}

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

	rw::common::Cache<GeometryData*, ODESimulator::TriMeshData > _cache;

//	ODESimulator::TriMeshData* buildTriMesh(dTriMeshDataID triMeshDataId, const std::vector<Frame*>& frames,
//			rw::kinematics::Frame* parent, const rw::kinematics::State &state, bool invert){

	ODESimulator::TriMeshData::Ptr buildTriMesh(GeometryData::Ptr gdata, const State &state, bool invert = false){
		// check if the geometry is allready in cache
		if( _cache.isInCache(gdata.get()) ){
			ODESimulator::TriMeshData::Ptr tridata = _cache.get(gdata.get());
			return tridata;
		}

        bool ownedData = false;
        RW_DEBUGS("indexed stuff");
        IndexedTriMesh<float>::Ptr imesh;

		// if not in cache then we need to create a TriMeshData geom,
		// but only if the geomdata is a trianglemesh
		if( !dynamic_cast<TriMesh*>(gdata.get()) ){
		    TriMesh::Ptr mesh = gdata->getTriMesh();
		    imesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >(*mesh,0.00001);
		    ownedData = true;
		} else if( !dynamic_cast< IndexedTriMesh<float>* >(gdata.get()) ){
			// convert the trimesh to an indexed trimesh
			RW_DEBUGS("to indexed tri mesh");
			imesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >(*((TriMesh*)gdata.get()),0.00001);
			ownedData = true;
		} else {
			imesh = static_cast< IndexedTriMesh<float>* >(gdata.get());
		}
		RW_DEBUGS("done casting");
		int nrOfVerts = imesh->getVertices().size();
		RW_DEBUGS("nr tris");
		int nrOfTris = imesh->getSize();

		// std::cout  << "- NR of faces: " << nrOfTris << std::endl;
		// std::cout  << "- NR of verts: " << nrOfVerts << std::endl;
		RW_DEBUGS("new ode trimesh");
		ODESimulator::TriMeshData *data =
			new ODESimulator::TriMeshData(nrOfTris*3, nrOfVerts*3);
		RW_DEBUGS("trimeshid");
		dTriMeshDataID triMeshDataId = dGeomTriMeshDataCreate();

		//const float myScale = 1.02;

		data->triMeshID = triMeshDataId;
		int vertIdx = 0;
		RW_DEBUGS("for each vertice");
		BOOST_FOREACH(const Vector3D<float>& v, imesh->getVertices()){
			data->vertices[vertIdx+0] = v(0);
			data->vertices[vertIdx+1] = v(1);
			data->vertices[vertIdx+2] = v(2);
			vertIdx+=3;
		}
		RW_DEBUGS("for each triangle");
		int indiIdx = 0;
		//BOOST_FOREACH(, imesh->getTriangles()){
		for(size_t i=0;i<imesh->getSize();i++){
			const IndexedTriangle<uint32_t> tri = imesh->getIndexedTriangle(i);
			if(invert){
				data->indices[indiIdx+0] = tri.getVertexIdx(2);
				data->indices[indiIdx+1] = tri.getVertexIdx(1);
				data->indices[indiIdx+2] = tri.getVertexIdx(0);
			} else {
				data->indices[indiIdx+0] = tri.getVertexIdx(0);
				data->indices[indiIdx+1] = tri.getVertexIdx(1);
				data->indices[indiIdx+2] = tri.getVertexIdx(2);
			}
			//if(data->indices[indiIdx+0]>=(size_t)nrOfVerts)
			//	std::cout << indiIdx+0 << " " << data->indices[indiIdx+0] << "<" << nrOfVerts << std::endl;
			RW_ASSERT( data->indices[indiIdx+0]< (size_t)nrOfVerts );
			//if(data->indices[indiIdx+1]>=(size_t)nrOfVerts)
			//	std::cout << data->indices[indiIdx+1] << "<" << nrOfVerts << std::endl;
			RW_ASSERT( data->indices[indiIdx+1]<(size_t)nrOfVerts );
			//if(data->indices[indiIdx+2]>=(size_t)nrOfVerts)
			//	std::cout << data->indices[indiIdx+2] << "<" << nrOfVerts << std::endl;
			RW_ASSERT( data->indices[indiIdx+2]<(size_t)nrOfVerts );

			indiIdx+=3;
		}
		RW_DEBUGS("build ode trimesh");
		dGeomTriMeshDataBuildSingle(triMeshDataId,
				&data->vertices[0], 3*sizeof(float), nrOfVerts,
				(dTriIndex*)&data->indices[0], nrOfTris*3, 3*sizeof(dTriIndex));

		RW_DEBUGS("DONE tristuff");
		// write all data to the disc
		/*
		std::ofstream fstr;
		std::stringstream sstr;
		sstr << "test_data_" << nrOfVerts << ".h";
		fstr.open(sstr.str().c_str());
		if(!fstr.is_open())
		    RW_THROW("fstr not open!");
		fstr << "const int VertexCount = " << nrOfVerts << "\n"
			 << "const int IndexCount = " << nrOfTris << " * 3\n"
			 << "\n\n"
			 << "float Vertices[VertexCount * 3] = {";
		for(int i=0;i<nrOfVerts-1;i++){
			fstr << data->vertices[i*3+0] << ","
				 << data->vertices[i*3+1] << ","
				 << data->vertices[i*3+2] << ",\n";
		}
		fstr << data->vertices[(nrOfVerts-1)*3+0] << ","
			 << data->vertices[(nrOfVerts-1)*3+1] << ","
			 << data->vertices[(nrOfVerts-1)*3+2] << "\n };";

		fstr << "\n\ndTriIndex Indices[IndexCount/3][3] = { \n";
		for(int i=0;i<nrOfTris-1;i++){
			fstr << "{" << data->indices[i*3+0] << ","
				 << data->indices[i*3+1] << ","
				 << data->indices[i*3+2] << "},\n";
		}
		fstr << "{" << data->indices[(nrOfTris-1)*3+0] << ","
			 << data->indices[(nrOfTris-1)*3+1] << ","
			 << data->indices[(nrOfTris-1)*3+2] << "}\n };";

		fstr.close();
*/
		//triMeshDatas.push_back(boost::shared_ptr<ODESimulator::TriMeshData>(data) );

//		if( ownedData )
//			delete imesh;

		return data;
	}

	std::vector<ODESimulator::TriGeomData*> buildTriGeom(Body *body, const State &state, dSpaceID spaceid, bool invert = false){
        RW_DEBUGS( "----- BEGIN buildTriGeom --------" );
		std::vector<Geometry::Ptr> geoms = body->getGeometry();
		RW_DEBUGS( "Nr of geoms: " << geoms.size() );
        std::vector<ODESimulator::TriGeomData*> triGeomDatas;
        for(size_t i=0; i<geoms.size(); i++){
            GeometryData::Ptr rwgdata = geoms[i]->getGeometryData();
            Transform3D<> transform = geoms[i]->getTransform();
            RW_DEBUGS(" TRANSFORM: " << geoms[i]->getTransform());

            ODESimulator::TriMeshData::Ptr triMeshData = buildTriMesh(rwgdata,state,invert);
            if(triMeshData==NULL){
            	continue;
            }
            dGeomID geoId;
            bool isTriMesh=false;
            if( Sphere* sphere_rw = dynamic_cast<Sphere*>(rwgdata.get()) ){
                geoId = dCreateSphere(spaceid, (dReal)sphere_rw->getRadius());
            } else if( Cylinder* cyl_rw = dynamic_cast<Cylinder*>(rwgdata.get()) ){
                geoId = dCreateCylinder(spaceid, (dReal)cyl_rw->getRadius(), (dReal)cyl_rw->getHeight());
            } else if( Plane* plane_rw = dynamic_cast<Plane*>(rwgdata.get()) ){
                Vector3D<> n = plane_rw->normal();
                geoId = dCreatePlane(spaceid, (dReal)n[0], (dReal)n[1], (dReal)n[2], (dReal)plane_rw->d());
            } else {
                geoId = dCreateTriMesh(spaceid, triMeshData->triMeshID, NULL, NULL, NULL);
                isTriMesh = true;
            }

    	    dGeomSetData(geoId, triMeshData->triMeshID);
    	    ODESimulator::TriGeomData *gdata = new ODESimulator::TriGeomData(triMeshData);
    	    gdata->isGeomTriMesh = isTriMesh;
    	    ODEUtil::toODETransform(transform, gdata->p, gdata->rot);
    	    gdata->t3d = transform;
            triGeomDatas.push_back(gdata);
            gdata->geomId = geoId;
        }
	    // create geo
        RW_DEBUGS( "----- END buildTriGeom --------");
		return triGeomDatas;
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

bool isInErrorGlobal = false;

ODESimulator::ODESimulator(DynamicWorkCell::Ptr dwc):
	_dwc(dwc),_time(0.0),_render(new ODEDebugRender(this)),
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
    _nextFeedbackIdx(0),
    _excludeMap(0,100),
    _oldTime(0)
{

    // verify that the linked ode library has the correct
    int isDouble = dCheckConfiguration ("ODE_double_precision");
    int isRelease = dCheckConfiguration ("ODE_EXT_no_debug");

    if(sizeof(dReal)==sizeof(double)){
        if(isDouble==0){
            RW_THROW("Current linked library does not support double! change dDOUBLE to dSINGLE or link agains ode lib with double support!");
        }
    } else {
        if(isDouble==1){
            RW_THROW("Current linked library does not support single! change dSINGLE to dDOUBLE or link agains ode lib with single support!");
        }
    }

}

void ODESimulator::setEnabled(Body* body, bool enabled){
    if(!body)
        RW_THROW("Body is NULL!");
    dBodyID dBody = _rwFrameToODEBody[ body->getBodyFrame() ];
    Frame *frame = _rwODEBodyToFrame[ dBody ];
    if( enabled ) {
        dBodyEnable(dBody);
        _enabledMap[*frame] = 1;
    } else{
        dBodyDisable(dBody);
        _enabledMap[*frame] = 0;
    }

}
namespace {
	bool isClose(dReal *m1, const dReal *P, const dReal *R, double eps ){
		double rsum = 0;
		for(int i=0;i<12;i++){
			float val = fabs(m1[i]-R[i]);
			rsum += val*val;
		}
		double psum = fabs(m1[12]-P[0])*fabs(m1[12]-P[0])+
					  fabs(m1[13]-P[1])*fabs(m1[13]-P[1])+
					  fabs(m1[14]-P[2])*fabs(m1[14]-P[2]);

		return rsum<eps && psum<eps;
	}
}


namespace {
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
	BOOST_FOREACH(dBodyID body, _allbodies){
		ODEStateStuff res;
		res.body = body;
		drealCopy( dBodyGetPosition(body), res.pos, 3);
		drealCopy( dBodyGetQuaternion(body), res.rot, 4);
		drealCopy( dBodyGetLinearVel  (body), res.lvel, 3);
		drealCopy( dBodyGetAngularVel (body), res.avel, 3);
		drealCopy( dBodyGetForce  (body), res.force, 3);
		drealCopy( dBodyGetTorque (body), res.torque, 3);
		_odeStateStuff.push_back(res);
	}

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

	BOOST_FOREACH(ODETactileSensor* sensor,_odeSensors){
		sensor->clear();
	}
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
    RW_DEBUGS("------------- Controller update:");
    double lastDt = _time-_oldTime;
    if(lastDt<=0)
        lastDt = dt;
    //std::cout << "Controller update" << std::endl;
    //// std::cout  << "Controller" << std::endl;
    BOOST_FOREACH(SimulatedController::Ptr controller, _controllers ){
        controller->update(lastDt, state);
    }
    //std::cout << "Device pre-update" << std::endl;
    RW_DEBUGS("------------- Device pre-update:");
    BOOST_FOREACH(ODEDevice *dev, _odeDevices){
        dev->update(lastDt, state);
    }
    //std::cout << "Body pre-update" << std::endl;
    RW_DEBUGS("------------- Body pre-update:");
    BOOST_FOREACH(ODEBody *body, _odeBodies){
        body->update(lastDt, state);
    }
    //std::cout << "Collisions at " << _time << std::endl;
    RW_DEBUGS("------------- Collisions at " << _time << " :");
	// Detect collision
    _allcontacts.clear();

#define USE_ROBWORK_CONTACT_RESOLVER 1
#if USE_ROBWORK_CONTACT_RESOLVER

    TIMING("Collision: ", detectCollisionsRW(state) );
    _allcontactsTmp = _allcontacts;

#else
	try {
		TIMING("Collision: ", dSpaceCollide(_spaceId, this, &nearCallback) );
		_allcontactsTmp = _allcontacts;
	} catch ( ... ) {
		std::cout << "Collision ERROR";
		Log::errorLog() << "******************** Caught exeption in collision function!*******************" << std::endl;
	}
#endif

	if(isInErrorGlobal){
	    dJointGroupEmpty(_contactGroupId);
	    // and the joint feedbacks that where used is also destroyed
	    _nextFeedbackIdx=0;

		RW_THROW("Collision error");
	}

	RW_DEBUGS("--------------------------- ");

	// Step world
	RW_DEBUGS("------------- Step dt=" << dt <<" at " << _time << " :");
	//std::cout << "------------- Step dt=" << dt <<" at " << _time << " :"<< std::endl;
	//std::cout << "StepMethod: " << _stepMethod << std::endl;
	//std::cout << "StepMethod: " << _maxIter << std::endl;
	//std::cout << "[";
	/*
	try {
		switch(_stepMethod){
		case(WorldStep): TIMING("Step: ", dWorldStep(_worldId, dt)); break;
		case(WorldQuickStep): TIMING("Step: ", dWorldQuickStep(_worldId, dt)); break;
		//case(WorldFast1): TIMING("Step: ", dWorldStepFast1(_worldId, dt, _maxIter)); break;
		default:
			TIMING("Step: ", dWorldStep(_worldId, dt));
		}
	} catch ( ... ) {
		std::cout << "ERROR";
		Log::errorLog() << "******************** Caught exeption in step function!*******************" << std::endl;
	}
	if(isInErrorGlobal){
	    dJointGroupEmpty(_contactGroupId);
	    // and the joint feedbacks that where used is also destroyed
	    _nextFeedbackIdx=0;

		RW_THROW("Collision error");
	}*/

	//std::cout << "]" << std::endl;

	saveODEState();
	double dttmp = dt;
	int i;
	const int MAX_TIME_ITERATIONS = 10;
	for(i=0;i<MAX_TIME_ITERATIONS;i++){
		//TIMING("Step: ", dWorldStep(_worldId, dttmp));
	    try {
	        switch(_stepMethod){
	        case(WorldStep): TIMING("Step: ", dWorldStep(_worldId, dttmp)); break;
	        case(WorldQuickStep): TIMING("Step: ", dWorldQuickStep(_worldId, dttmp)); break;
	        //case(WorldFast1): TIMING("Step: ", dWorldStepFast1(_worldId, dt, _maxIter)); break;
	        default:
	            TIMING("Step: ", dWorldStep(_worldId, dttmp));
	        }
	    } catch ( ... ) {
	        std::cout << "ERROR";
	        Log::errorLog() << "******************** Caught exeption in step function!*******************" << std::endl;
	    }


		// this is onlu done to check that the stepsize was not too big
		//TIMING("Collision: ", dSpaceCollide(_spaceId, this, &nearCallback) );
		bool inCollision = false;
		TIMING("Collision Resolution: ", inCollision = detectCollisionsRW(state, true) );
		if(!inCollision){
		    //std::cout << "THERE IS NO PENETRATION" << std::endl;
			break;
		} else if( i==MAX_TIME_ITERATIONS-1){
		    RW_WARN("PENETRATIONS...");
		    break;
		}
		// max penetration was then we step back to the last configuration and we try again
		dttmp /= 2;

		restoreODEState();

	}
	_oldTime = _time;
	_time += dttmp;

	RW_DEBUGS("------------- Device post update:");
	//std::cout << "Device post update:" << std::endl;
	BOOST_FOREACH(ODEDevice *dev, _odeDevices){
	    dev->postUpdate(state);
	}
	RW_DEBUGS("------------- Update robwork bodies:");
	//std::cout << "Update robwork bodies:" << std::endl;
    // now copy all state info into state/bodies (transform,vel,force)
    for(size_t i=0; i<_odeBodies.size(); i++){
        _odeBodies[i]->postupdate(state);
    }

    RW_DEBUGS("------------- Sensor update :");
    //std::cout << "Sensor update :" << std::endl;
    // update all sensors with the values of the joints
    BOOST_FOREACH(ODETactileSensor *odesensor, _odeSensors){
        odesensor->update(dttmp, state);
    }
    RW_DEBUGS("- removing joint group");
    // Remove all temporary collision joints now that the world has been stepped
    dJointGroupEmpty(_contactGroupId);
    // and the joint feedbacks that where used is also destroyed
    _nextFeedbackIdx=0;

	RW_DEBUGS("------------- Update trimesh prediction:");
	BOOST_FOREACH(TriGeomData *data, _triGeomDatas){
	    if(!data->isGeomTriMesh)
	        continue;
	    dGeomID geom = data->geomId;
	    //if( dGeomGetClass(geom) != dTriMeshClass )
	    //    continue;
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

dBodyID ODESimulator::createRigidBody(Body* rwbody,
                                      const BodyInfo& info,
                                      const rw::kinematics::State& state,
                                      dSpaceID spaceid){
    // create a triangle mesh for all staticly connected nodes
    // std::vector<Frame*> frames = DynamicUtil::getAnchoredFrames( *bframe, state);
    RW_DEBUGS( "- Create Rigid body: " << rwbody->getBodyFrame()->getName());

	std::vector<TriGeomData*> gdatas = buildTriGeom(rwbody, state, spaceid, false);

	if(gdatas.size()==0){
		RW_THROW("Body: "<< rwbody->getBodyFrame()->getName() << " has no geometry!");
	}

    Vector3D<> mc = info.masscenter;
    dMass m;
    setODEBodyMass(&m, info.mass, Vector3D<>(0,0,0), info.inertia);
    //std::cout << "RW inertia: " << info.inertia << std::endl;
    //printMassInfo(m, *rwbody->getBodyFrame() );
    dMassCheck(&m);
    // create the body and initialize mass, inertia and stuff
    dBodyID bodyId = dBodyCreate(_worldId);
	Transform3D<> wTb = Kinematics::worldTframe(rwbody->getBodyFrame(), state);
	wTb.P() += wTb.R()*mc;
	ODEUtil::setODEBodyT3D(bodyId, wTb);

	dBodySetMass(bodyId, &m);

    int mid = _materialMap.getDataID( info.material );
    int oid = _contactMap.getDataID( info.objectType );

	ODEBody *odeBody=0;
    if(RigidBody *rbody = dynamic_cast<RigidBody*>(rwbody)){
        odeBody = new ODEBody(bodyId, rbody, info.masscenter, mid, oid);
        _odeBodies.push_back(odeBody);
        _allbodies.push_back(bodyId);
        dBodySetData (bodyId, (void*)odeBody);
    } else if(RigidJoint *rjbody = dynamic_cast<RigidJoint*>(rwbody)){
        odeBody = new ODEBody(bodyId, rjbody, info.masscenter, mid, oid);
        dBodySetData (bodyId, (void*)odeBody);
        _allbodies.push_back(bodyId);
    }

	// now associate all geometry with the body
	BOOST_FOREACH(TriGeomData* gdata, gdatas){
		_triGeomDatas.push_back(gdata);
		//Vector3D<> mc = gdata->t3d.R() * bmc;
		dGeomSetBody(gdata->geomId, bodyId);
		dGeomSetData(gdata->geomId, odeBody);
		_frameToOdeGeoms[rwbody->getBodyFrame()] = gdata->geomId;
		// the geom must be attached to body before offset is possible
		dGeomSetOffsetPosition(gdata->geomId, gdata->p[0]-mc[0], gdata->p[1]-mc[1], gdata->p[2]-mc[2]);
		dGeomSetOffsetQuaternion(gdata->geomId, gdata->rot);
	}
	dBodySetMaxAngularSpeed(bodyId, 10);
    _rwODEBodyToFrame[bodyId] = rwbody->getBodyFrame();
    _rwFrameToODEBody[rwbody->getBodyFrame()] = bodyId;
    BOOST_FOREACH(Frame* frame, rwbody->getFrames()){
        //std::cout  << "--> Adding frame: " << frame->getName() << std::endl;
        _rwFrameToODEBody[frame] = bodyId;
    }

    return bodyId;
}

rwsim::drawable::SimulatorDebugRender* ODESimulator::createDebugRender(){
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

    PER BODY OPTIONS

    PER material-pair OPTIONS
        friction - coloumb,
        restitution -

     */


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

dBodyID ODESimulator::createKinematicBody(KinematicBody* kbody,
        const BodyInfo& info,
        const rw::kinematics::State& state,
        dSpaceID spaceid)
{
    RW_ASSERT(kbody!=NULL);
    // create a triangle mesh for all statically connected nodes
    std::vector<TriGeomData*> gdatas = buildTriGeom(kbody, state, _spaceId, false);
    // if no triangles was loaded then continue
    if( gdatas.size()==0 ){
        RW_THROW("No triangle mesh defined for this body: " << kbody->getBodyFrame()->getName());
    }

    Vector3D<> mc = info.masscenter;
    Transform3D<> wTb = Kinematics::worldTframe(kbody->getBodyFrame(), state);
    wTb.P() += wTb.R()*mc;

    dBodyID bodyId = dBodyCreate(_worldId);
    dBodySetKinematic(bodyId);
    ODEUtil::setODEBodyT3D(bodyId, wTb);

    int mid = _materialMap.getDataID( info.material );
    int oid = _contactMap.getDataID( info.objectType );

    ODEBody *odeBody = new ODEBody(bodyId, kbody, mid , oid);
    _odeBodies.push_back(odeBody);
    dBodySetData (bodyId, odeBody);
    _allbodies.push_back(bodyId);
    _rwODEBodyToFrame[bodyId] = kbody->getBodyFrame();
    _rwFrameToODEBody[kbody->getBodyFrame()] = bodyId;


    BOOST_FOREACH(TriGeomData* gdata, gdatas){


        _triGeomDatas.push_back(gdata);
        // set position and rotation of body
        dGeomSetBody(gdata->geomId, bodyId);

        //ODEBody *odeBody = new ODEBody(gdata->geomId, &kbody->getBodyFrame(), mid , oid);
        dGeomSetData(gdata->geomId, odeBody);

        _frameToOdeGeoms[kbody->getBodyFrame()] = gdata->geomId;

        dGeomSetOffsetPosition(gdata->geomId, gdata->p[0]-mc[0], gdata->p[1]-mc[1], gdata->p[2]-mc[2]);
        dGeomSetOffsetQuaternion(gdata->geomId, gdata->rot);
    }


    BOOST_FOREACH(Frame* frame, kbody->getFrames()){
        RW_DEBUGS( "(KB) --> Adding frame: " << frame->getName() );
        _rwFrameToODEBody[frame] = bodyId;
    }
    return bodyId;
}

dBodyID ODESimulator::createFixedBody(Body* rwbody,
        const BodyInfo& info,
        const rw::kinematics::State& state,
        dSpaceID spaceid)
{
	FixedBody *rbody = dynamic_cast<FixedBody*>( rwbody );
	if(rbody==NULL)
		RW_THROW("Not a fixed body!");
	// create a triangle mesh for all statically connected nodes
	std::vector<TriGeomData*> gdatas = buildTriGeom(rwbody, state, _spaceId, false);
	// if no triangles was loaded then continue
	if( gdatas.size()==0 ){
		RW_THROW("No triangle mesh defined for this body: " << rwbody->getBodyFrame()->getName());
	}

    Vector3D<> mc = info.masscenter;
    Transform3D<> wTb = Kinematics::worldTframe(rbody->getBodyFrame(), state);
    //wTb.P() += wTb.R()*mc;

	//dBodyID bodyId = dBodyCreate(_worldId);
	//dBodySetKinematic(bodyId);
	//ODEUtil::setODEBodyT3D(bodyId, wTb);

	BOOST_FOREACH(TriGeomData* gdata, gdatas){
	    int mid = _materialMap.getDataID( info.material );
        int oid = _contactMap.getDataID( info.objectType );

		_triGeomDatas.push_back(gdata);
		// set position and rotation of body
		//dGeomSetBody(gdata->geomId, bodyId);

		ODEBody *odeBody = new ODEBody(gdata->geomId, rbody, mid , oid);
		dGeomSetData(gdata->geomId, odeBody);
		Transform3D<> gt3d = wTb*gdata->t3d;
		ODEUtil::setODEGeomT3D(gdata->geomId, gt3d);

		_frameToOdeGeoms[rbody->getBodyFrame()] = gdata->geomId;
        //dGeomSetOffsetPosition(gdata->geomId, gdata->p[0]-mc[0], gdata->p[1]-mc[1], gdata->p[2]-mc[2]);
        //dGeomSetOffsetQuaternion(gdata->geomId, gdata->rot);
	}

    //dBodySetData (bodyId, 0);
    //_allbodies.push_back(bodyId);
	//_rwODEBodyToFrame[bodyId] = &rwbody->getBodyFrame();
	//_rwFrameToODEBody[&rwbody->getBodyFrame()] = bodyId;
	//BOOST_FOREACH(Frame* frame, rwbody->getFrames()){
	//	std::cout  << "--> Adding frame: " << frame->getName() << std::endl;
	//	_rwFrameToODEBody[frame] = bodyId;
	//}
	return 0;
}

namespace {
	void EmptyMessageFunction(int errnum, const char *msg, va_list ap){
		//char str[400];
		//sprintf(str, msg, *ap);
		//std::cout << "ODE internal Message: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"" << std::endl;
		//Log::infoLog() << "ODE internal msg: errnum=" << errnum << " odemsg=\"" << msg<< "\"\n";
		//RW_WARN("ODE internal msg: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"");
		//isInErrorGlobal=true;
	}

	void ErrorMessageFunction(int errnum, const char *msg, va_list ap){
		//char str[400];
		//sprintf(str, msg, *ap);
		std::cout << "ODE internal Error: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"" << std::endl;
		isInErrorGlobal=true;
		//RW_THROW("ODE internal Error: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"");
	}

	void DebugMessageFunction(int errnum, const char *msg, va_list ap){
		char str[400];
		//sprintf(str, msg, *ap);
		std::cout << "ODE internal Debug: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"" << std::endl;
		isInErrorGlobal=true;
		//dWorldCleanupWorkingMemory();

		RW_THROW("ODE internal Debug: errnum=" << errnum << " odemsg=\"" <<  msg<< "\"");
	}

}

void ODESimulator::initPhysics(rw::kinematics::State& state)
{
    _propertyMap = _dwc->getEngineSettings();
    CollisionSetup cSetup = Proximity::getCollisionSetup( *_dwc->getWorkcell() );
    FramePairList excludeList = BasicFilterStrategy::getExcludePairList(*_dwc->getWorkcell(), cSetup);
    BOOST_FOREACH(rw::kinematics::FramePair& pair, excludeList){
        _excludeMap[rw::kinematics::FramePair(pair.first,pair.second)] = 1;
    }

    _bpstrategy = ownedPtr( new BasicFilterStrategy( _dwc->getWorkcell() ) );
    // build the frame map
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
        }
	}

	// Create joint group
    _contactGroupId = dJointGroupCreate(0);

	// add gravity
    Vector3D<> gravity = _dwc->getGravity();
	dWorldSetGravity ( _worldId, gravity(0), gravity(1), gravity(2) );
	dWorldSetCFM ( _worldId, _worldCFM );
	dWorldSetERP ( _worldId, _worldERP );

	dWorldSetContactSurfaceLayer(_worldId, 0.0001);
	//dWorldSetContactMaxCorrectingVel(_worldId, 0.05);
	//dWorldSetAngularDamping()
    State initState = state;
    // first set the initial state of all devices.
    BOOST_FOREACH(DynamicDevice* device, _dwc->getDynamicDevices() ){
        JointDevice *jdev = dynamic_cast<JointDevice*>( &(device->getModel()) );
        if(jdev==NULL)
            continue;
        Q offsets = Q::zero( jdev->getQ(state).size() );
        jdev->setQ( offsets , initState );
    }

    //dCreatePlane(_spaceId,0,0,1,0);

	// convert collision geometries from DynamicWorkcell form to ODE form
	{
		// create collision shapes for all bodies in dynamic workcell

		// TODO: add all rigid bodies from dynamic workcell to the ODE dynamicworld
		std::vector<Body*> bodies = _dwc->getBodies();
		for(size_t i=0; i<bodies.size(); i++){
			if( dynamic_cast<RigidBody*>( bodies[i] ) ){
				RigidBody *rbody = dynamic_cast<RigidBody*>( bodies[i] );
                dBodyID bodyId = createRigidBody(bodies[i], rbody->getInfo(), initState, _spaceId);

                _bodies.push_back(bodyId);
                //_rwBodies.push_back(rbody);
                dBodySetAutoDisableFlag(bodyId, 1);
			} else if( dynamic_cast<KinematicBody*>( bodies[i] ) ) {
				KinematicBody *kbody = dynamic_cast<KinematicBody*>( bodies[i] );
				createKinematicBody(kbody, kbody->getInfo(), initState, _spaceId);
			} else if( dynamic_cast<FixedBody*>( bodies[i] ) ) {
				FixedBody *fbody = dynamic_cast<FixedBody*>( bodies[i] );
				createFixedBody(fbody, fbody->getInfo(), initState, _spaceId);
			}
		}
	}

	Frame *wframe = _dwc->getWorkcell()->getWorldFrame();
	_rwODEBodyToFrame[0] = wframe;


   BOOST_FOREACH(DynamicDevice* device, _dwc->getDynamicDevices() ){
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
            if(_rwFrameToODEBody.find(baseParent) == _rwFrameToODEBody.end()){
                RW_WARN("No parent data available, connecting base to world!");
            	dBodyID baseParentBodyId = dBodyCreate(_worldId);
            	ODEUtil::setODEBodyT3D(baseParentBodyId, Kinematics::worldTframe(baseParent,initState));
            	_rwFrameToODEBody[baseParent] = baseParentBodyId;
            }
            dBodyID baseParentBodyID = _rwFrameToODEBody[baseParent];

            // now create the base
            RW_DEBUGS( "Create base");
            dBodyID baseBodyID = 0;
            if( dynamic_cast<FixedBody*>(baseBody) ){
                RW_DEBUGS("- Fixed");
            	baseBodyID = createFixedBody(baseBody, baseBody->getInfo(), state, space);
            } else if(KinematicBody *kbody = dynamic_cast<KinematicBody*>(baseBody)){
                RW_DEBUGS("- Kinematic");
            	//kbody->getInfo().print();
            	baseBodyID = createKinematicBody(kbody, kbody->getInfo(), state, space);
            } else if(dynamic_cast<RigidBody*>(baseBody)){
                RW_DEBUGS("- Rigid");
            	baseBodyID = createRigidBody(baseBody, baseBody->getInfo(), state, space);
            } else {
            	RW_THROW("Unknown body type of robot \""<< device->getModel().getName()<<"\" base");
            }
            _rwFrameToODEBody[ base ] = baseBodyID;

            // and connect the base to the parent using a fixed joint if the base is rigid
            if( dynamic_cast<RigidBody*>(baseBody) ){
				dJointID baseJoint = dJointCreateFixed(_worldId, 0);
	            dJointAttach(baseJoint, baseBodyID, baseParentBodyID);
	            dJointSetFixed(baseJoint);
            }

            std::vector<ODEJoint*> odeJoints;
            Q maxForce = fDev->getForceLimit();
            RW_DEBUGS("BASE:" << base->getName() << "<--" << base->getParent()->getName() );

            size_t i =0;
            BOOST_FOREACH(RigidJoint *rjoint, fDev->getBodies() ){
                Frame *joint = rjoint->getBodyFrame();
                Frame *parent = joint->getParent(initState);
                RW_DEBUGS(parent->getName() << "<--" << joint->getName());

                dBodyID odeParent;// = _rwFrameToODEBody[parent];
                Frame *parentFrame = NULL;
                if(_rwFrameToODEBody.find(parent) == _rwFrameToODEBody.end() ){
                    // could be that the reference is
                    RW_WARN("odeParent is NULL, " << joint->getName() << "-->"
							<< parent->getName()
							<< " Using WORLD as parent");

                    odeParent = _rwFrameToODEBody[wframe];
                    _rwFrameToODEBody[parent] = odeParent;
                }
                odeParent = _rwFrameToODEBody[parent];
                parentFrame = _rwODEBodyToFrame[odeParent];

                dBodyID odeChild = createRigidBody(rjoint, rjoint->getInfo() , initState, space ); //_rwFrameToBtBody[joint];
                if(odeChild==NULL){
                    RW_WARN("odeChild is NULL, " << joint->getName());
                    RW_ASSERT(0);
                }

                Transform3D<> wTchild = Kinematics::worldTframe(joint,initState);
                Vector3D<> haxis = wTchild.R() * Vector3D<>(0,0,1);
                Vector3D<> hpos = wTchild.P();
                //Transform3D<> wTparent = Kinematics::WorldTframe(parentFrame,initState);

                 if(RevoluteJoint *rwjoint = dynamic_cast<RevoluteJoint*>(joint)){
                     RW_DEBUGS("Revolute joint");
                     const double qinit = rwjoint->getData(initState)[0];

                     dJointID hinge = dJointCreateHinge (_worldId, 0);
                     dJointAttach(hinge, odeChild, odeParent);
                     dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                     dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

                     dJointID motor = dJointCreateAMotor (_worldId, 0);
                     dJointAttach(motor, odeChild, odeParent);
                     dJointSetAMotorNumAxes(motor, 1);
                     dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                     dJointSetAMotorAngle(motor,0, qinit);
                     dJointSetAMotorParam(motor,dParamFMax, maxForce(i) );
                     dJointSetAMotorParam(motor,dParamVel,0);

                     // we use motor to simulate friction
                     /*dJointID motor2 = dJointCreateAMotor (_worldId, 0);
                     dJointAttach(motor2, odeChild, odeParent);
                     dJointSetAMotorNumAxes(motor2, 1);
                     dJointSetAMotorAxis(motor2, 0, 1, haxis(0) , haxis(1), haxis(2));
                     dJointSetAMotorAngle(motor2,0, qinit);
                     dJointSetAMotorParam(motor2,dParamFMax, maxForce(i)/50 );
                     dJointSetAMotorParam(motor2,dParamVel,0);
					*/
                     //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                     //dJointSetAMotorParam(Amotor,dParamHiStop,0);
                     ODEJoint *odeJoint = new ODEJoint(ODEJoint::Revolute, hinge, motor, odeChild, rjoint);
                     _jointToODEJoint[rwjoint] = odeJoint;
                     odeJoints.push_back(odeJoint);
                     _allODEJoints.push_back(odeJoint);
                 } else if( dynamic_cast<DependentRevoluteJoint*>(joint)){
                     RW_DEBUGS("DependentRevolute");
                     DependentRevoluteJoint *rframe = dynamic_cast<DependentRevoluteJoint*>(joint);
                     Joint *owner = &rframe->getOwner();
                     const double qinit = owner->getData(initState)[0]*rframe->getScale()+0;

                     dJointID hinge = dJointCreateHinge (_worldId, 0);
                     dJointAttach(hinge, odeChild, odeParent);
                     dJointSetHingeAxis(hinge, haxis(0) , haxis(1), haxis(2));
                     dJointSetHingeAnchor(hinge, hpos(0), hpos(1), hpos(2));

                     dJointID motor = dJointCreateAMotor (_worldId, 0);
                     dJointAttach(motor, odeChild, odeParent);
                     dJointSetAMotorNumAxes(motor, 1);
                     dJointSetAMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                     dJointSetAMotorAngle(motor,0, qinit);
                     dJointSetAMotorParam(motor,dParamFMax, 20/*maxForce(i)*/ );
                     dJointSetAMotorParam(motor,dParamVel,0);



                     ODEJoint *odeOwner = _jointToODEJoint[owner];
                     ODEJoint *odeJoint = new ODEJoint( ODEJoint::Revolute, hinge, motor,  odeChild,
                                                        odeOwner, rframe ,
                                                        rframe->getScale(), 0 , rjoint);
                     odeJoints.push_back(odeJoint);
                     //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                     //dJointSetAMotorParam(Amotor,dParamHiStop,0);
                     _allODEJoints.push_back(odeJoint);
                 } else if( PrismaticJoint *pjoint = dynamic_cast<PrismaticJoint*>(joint) ){

                     // test if another joint is dependent on this joint


                     const double qinit = pjoint->getData(initState)[0];
                     dJointID slider = dJointCreateSlider (_worldId, 0);
                     dJointAttach(slider, odeChild, odeParent);
                     dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));
                     //dJointSetHingeAnchor(slider, hpos(0), hpos(1), hpos(2));

                     dJointID motor = dJointCreateLMotor (_worldId, 0);
                     dJointAttach(motor, odeChild, odeParent);
                     dJointSetLMotorNumAxes(motor, 1);
                     dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));
                     //dJointSetLMotorAngle(motor,0, qinit);

                     dJointSetLMotorParam(motor,dParamFMax, maxForce(i) );
                     dJointSetLMotorParam(motor,dParamVel,0);

                     //dJointSetAMotorParam(Amotor,dParamLoStop,-0);
                     //dJointSetAMotorParam(Amotor,dParamHiStop,0);
                     ODEJoint *odeJoint = new ODEJoint(ODEJoint::Prismatic, slider, motor, odeChild, rjoint);
                     _jointToODEJoint[pjoint] = odeJoint;
                     odeJoints.push_back(odeJoint);
                     _allODEJoints.push_back(odeJoint);

                 } else if( dynamic_cast<DependentPrismaticJoint*>(joint) ) {
                     RW_DEBUGS("DependentPrismaticJoint");
                     DependentPrismaticJoint *pframe = dynamic_cast<DependentPrismaticJoint*>(joint);
                     Joint *owner = &pframe->getOwner();
                     const double qinit = owner->getData(initState)[0]*pframe->getScale()+0;

                     dBodyID ownerBody = _rwFrameToODEBody[owner];

                     dJointID slider = dJointCreateSlider (_worldId, 0);
                     //dJointAttach(slider, odeChild, odeParent);
                     dJointAttach(slider, odeChild, ownerBody);
                     dJointSetSliderAxis(slider, haxis(0) , haxis(1), haxis(2));

                     dJointID motor = dJointCreateLMotor (_worldId, 0);
                     //dJointAttach(motor, odeChild, odeParent);
                     dJointAttach(motor, odeChild, ownerBody);
                     dJointSetLMotorNumAxes(motor, 1);
                     dJointSetLMotorAxis(motor, 0, 1, haxis(0) , haxis(1), haxis(2));

                    // std::cout << "i:" << i << " mforce_len: " << maxForce.size() << std::endl;
                     // TODO: should take the maxforce value of the owner joint
                     dJointSetLMotorParam(motor,dParamFMax, maxForce(i) );
                     dJointSetLMotorParam(motor,dParamVel,0);

                     ODEJoint *odeOwner = _jointToODEJoint[owner];
                     ODEJoint *odeJoint = new ODEJoint( ODEJoint::Prismatic, slider, motor, odeChild,
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


        } else  if( dynamic_cast<KinematicDevice*>( device ) ){
            RW_DEBUGS("KinematicDevice");
            // TODO: create all joints and make them kinematic
            KinematicDevice* kdev = dynamic_cast<KinematicDevice*>( device );
            dSpaceID space = dHashSpaceCreate( _spaceId );
            std::vector<dBodyID> kDevBodies;
            BOOST_FOREACH(KinematicBody *kbody, kdev->getBodies() ){
                dBodyID odeBodyID = createKinematicBody(kbody, kbody->getInfo(), state, space);
                kDevBodies.push_back(odeBodyID);
            }

            ODEKinematicDevice *odekdev = new ODEKinematicDevice( kdev, kDevBodies);
            _odeDevices.push_back(odekdev);

        } else {
            RW_WARN("Controller not supported!");
        }
    }

    RW_DEBUGS( "- ADDING SENSORS " );
    BOOST_FOREACH(rwlibs::simulation::SimulatedSensor::Ptr sensor, _dwc->getSensors()){
    	addSensor(sensor);
	}

    RW_DEBUGS( "- ADDING CONTROLLERS " );
    BOOST_FOREACH(rwlibs::simulation::SimulatedController::Ptr controller, _dwc->getControllers()){
    	addController(controller);
	}

    RW_DEBUGS( "- CREATING MATERIAL MAP " );
    _odeMaterialMap = new ODEMaterialMap(_materialMap, _contactMap, _odeBodies);

    RW_DEBUGS( "- RESETTING SCENE " );
	resetScene(state);
}

void ODESimulator::addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){
	_sensors.push_back(sensor);

	SimulatedSensor *ssensor = sensor.get();
    if( dynamic_cast<SimulatedTactileSensor*>(ssensor) ){
        SimulatedTactileSensor *tsensor = dynamic_cast<SimulatedTactileSensor*>(sensor.get());
        Frame *bframe = tsensor->getSensor()->getFrame();

        if( _rwFrameToODEBody.find(bframe)== _rwFrameToODEBody.end()){
            RW_THROW("The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics!");
        }

        ODETactileSensor *odesensor = new ODETactileSensor(tsensor);

        dBodyID odeBody = _rwFrameToODEBody[bframe];
        // TODO: this should be a list of sensors for each body/frame
        _odeBodyToSensor[odeBody] = odesensor;

        _odeSensors.push_back(odesensor);
    }
}

using namespace rw::proximity;

bool ODESimulator::detectCollisionsRW(rw::kinematics::State& state, bool onlyTestPenetration){
    //
    //std::cout << "detectCollisionsRW" << onlyTestPenetration << std::endl;
    ProximityFilter::Ptr filter = _bpstrategy->update(state);
    FKTable fk(state);
    ProximityStrategyData data;

    // next we query the BP filter for framepairs that are possibly in collision
    while( !filter->isEmpty() ){
        const FramePair& pair = filter->frontAndPop();

        // and lastly we use the dispatcher to find the strategy the
        // is required to compute the narrowphase collision
        const ProximityModel::Ptr &a = _frameToModels[*pair.first];
        const ProximityModel::Ptr &b = _frameToModels[*pair.second];
        if(a==NULL || b==NULL)
            continue;

        //std::cout << "bodies" << std::endl;
        dBodyID a_body = _rwFrameToODEBody[pair.first];
        dBodyID b_body = _rwFrameToODEBody[pair.second];

        //std::cout << "geoms" << std::endl;
        dGeomID a_geom = _frameToOdeGeoms[pair.first];
        dGeomID b_geom = _frameToOdeGeoms[pair.second];
        if(a_geom==NULL || b_geom==NULL)
            continue;


        ODEBody *a_data;
        if( a_body==NULL ) {
            a_data = (ODEBody*) dGeomGetData(a_geom);
        } else {
            a_data = (ODEBody*) dBodyGetData(a_body);
        }
        RW_DEBUGS("- get data2")
        ODEBody *b_data;
        if( b_body==NULL ) {
            b_data = (ODEBody*) dGeomGetData(b_geom);
        } else {
            b_data = (ODEBody*) dBodyGetData(b_body);
        }

        //const Transform3D<> aT = fk.get(*pair.first);
        //const Transform3D<> bT = fk.get(*pair.second);

        //std::cout << pair.first->getName() << " " << pair.second->getName() << std::endl;
        //const Transform3D<> aT = ODEUtil::getODEGeomT3D(a_geom);
        //const Transform3D<> bT = ODEUtil::getODEGeomT3D(b_geom);
        const Transform3D<> aT = a_data->getTransform();
        const Transform3D<> bT = b_data->getTransform();

        // first make standard collision detection, if in collision then compute all contacts from dist query

        //std::cout << "aT" << aT << std::endl;
        //std::cout << "bT" << bT << std::endl;
        const double MAX_PENETRATION = 0.0002;
        const double MAX_SEP_DISTANCE = 0.0002;
        MultiDistanceResult *res;

        if(onlyTestPenetration){

            data.setCollisionQueryType(FirstContact);
            bool collides = _narrowStrategy->inCollision(a, aT, b, bT, data);
            if(collides){
                return true;
            }
            continue;
        }

        /*
        // there is a collision and we need to find the correct contact points/normals.
        BodyBodyContact& bcon = _lastNonCollidingTransform[pair];
        if( bcon.firstContact ){
            std::cout << "First contact " << std::endl;
            data.setCollisionQueryType(FirstContact);
            bool collides = _narrowStrategy->inCollision(a, aT, b, bT, data);

            if( !collides ){
                // save the last transform such that we are able to find the normal in case the bodies penetrate in the next timestep.
                _lastNonCollidingTransform[pair] = BodyBodyContact(aT, bT);
                return;
            }

            std::cout << "_aT" << bcon.aT << std::endl;
            std::cout << "_bT" << bcon.bT << std::endl;

            bcon.firstContact = false;
        } else {
            // update the contact normal and transforms
            // we use the old contact normal to project the poses of the object into a non-penetrating stance

            // TODO: here we should try and resolve the point of contact.
            bcon.aT = aT; // translated in the negative normal direction
            bcon.aT.P() += MAX_SEP_DISTANCE*bcon.cnormal;
            bcon.bT = bT;
            std::cout << "_aT" << bcon.aT << std::endl;
            std::cout << "_bT" << bcon.bT << std::endl;
            std::cout << "ASSERT if in collision" << std::endl;
            RW_ASSERT(_narrowStrategy->inCollision(a, bcon.aT, b, bcon.bT, data)==false);
            std::cout << "after ASSERT if in collision" << std::endl;
        }



        // calculate the contacts
        data.setCollisionQueryType(AllContacts);
        res = &_narrowStrategy->distances(a, bcon.aT, b, bcon.bT, MAX_SEP_DISTANCE, data);
        */

        // TODO: if the object is a soft object then we need to add more contacts
        bool softcontact = true;
        double softlayer = 0.0;
        if( softcontact ){
            // change MAX_SEP_DISTANCE
            softlayer = 0.002;
        }

        data.setCollisionQueryType(AllContacts);
        res = &_narrowStrategy->distances(a, aT, b, bT, MAX_SEP_DISTANCE+softlayer, data);


        //if(res->distances.size()==0){
        //    bcon.firstContact = true;
        //}
        //std::cout << "aT " << aT << std::endl;
        //std::cout << "bT " << bT << std::endl;

        //data.setCollisionQueryType(AllContacts);
        //MultiDistanceResult &res = _narrowStrategy->distances(a, aT, b, bT, MAX_PENETRATION, data);
        //std::cout << "-- DISTANCES: " << res->distances.size() << "\n\n\n"<< std::endl;

        // create all contacts
        int numc = res->distances.size();
        if(_rwcontacts.size()<numc){
            _rwcontacts.resize(numc);
            _contacts.resize(numc);
        }
        int ni = 0;
        //std::cout << "--- {";
        //for(int i=0;i<numc;i++){
        //    std::cout << res.distances[i] << ", ";
        //}
        //std::cout << "}"<< std::endl;

        for(int i=0;i<numc;i++){

            if(res->distances[i]<0.00000001)
                continue;

            dContact &con = _contacts[ni];
            Vector3D<> p1 = aT * res->p1s[i];
            Vector3D<> p2 = aT * res->p2s[i];
            double len = (p2-p1).norm2();
            Vector3D<> n = (p2-p1)/(-len);
            //std::cout << "n: " << n << "\n";
            Vector3D<> p = n*(res->distances[i]/2) + p1;

            ODEUtil::toODEVector(n, con.geom.normal);
            ODEUtil::toODEVector(p, con.geom.pos);

            if( softcontact ){
                // scale the distances to fit into MAX_SEP_DISTANCE
                res->distances[i] *= MAX_SEP_DISTANCE/(MAX_SEP_DISTANCE+softlayer);
            }
            double penDepth = MAX_SEP_DISTANCE-(res->distances[i]+(MAX_SEP_DISTANCE-MAX_PENETRATION));
            con.geom.depth = penDepth;

            con.geom.g1 = a_geom;
            con.geom.g2 = b_geom;

            // calculate the friction direction between the bodies
            ODEUtil::toODEVector( Vector3D<>(0,1,0), con.fdir1 );
            ni++;
        }
        numc = ni;

        //bcon.cnormal =
            addContacts(numc, a_body, b_body, a_geom, b_geom, pair.first, pair.second);
        res->clear();
        // update the contact normal using the manifolds
    }

    if(onlyTestPenetration){
        return false;
    }
    return false;
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
                        (_contacts.size()-1) , &_contacts[0].geom,
                        sizeof(dContact));

    if( numc >= (int)_contacts.size()-1 ){
        numc = _contacts.size()-2;
        RW_WARN( "------- Too small collision buffer ------" );
    }

    addContacts(numc, b1, b2, o1, o2, frameB1, frameB2);
}

rw::math::Vector3D<> ODESimulator::addContacts(int numc, dBodyID b1, dBodyID b2, dGeomID o1, dGeomID o2, Frame *f1, Frame *f2){
    if(numc==0){
        RW_DEBUGS("No collisions detected!");
        return Vector3D<>(0,0,0);
    }

    ODEBody *dataB1;
    if( b1==NULL ) {
        dataB1 = (ODEBody*) dGeomGetData(o1);
    } else {
        dataB1 = (ODEBody*) dBodyGetData(b1);
    }

    ODEBody *dataB2;
    if( b2==NULL ) {
        dataB2 = (ODEBody*) dGeomGetData(o2);
    } else {
        dataB2 = (ODEBody*) dBodyGetData(b2);
    }


    //RW_DEBUGS("- detected: " << frameB1->getName() << " " << frameB2->getName());

    // check if any of the bodies are actually sensors
    ODETactileSensor *odeSensorb1 = _odeBodyToSensor[b1];
    ODETactileSensor *odeSensorb2 = _odeBodyToSensor[b2];

    // perform contact clustering
    //double threshold = std::min(dataB1->getCRThres(), dataB2->getCRThres());
    //std::cout << "Numc: " << numc << std::endl;



    int j=0;
    for(int i=0;i<numc;i++){
        ContactPoint &point = _rwcontacts[j];
        const dContact &con = _contacts[i];

        //if(con.geom.depth>0.01)
        //    continue;

        point.n = normalize( toVector3D(con.geom.normal) );
        point.p = toVector3D(con.geom.pos);
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
    if(_srcIdx.size()<numc)
        _srcIdx.resize(numc);
    if(_dstIdx.size()<numc)
        _dstIdx.resize(numc);

    int fnumc = ContactCluster::normalThresClustering(
									&_rwcontacts[0], numc,
                                    &_srcIdx[0], &_dstIdx[0],
                                    &_rwClusteredContacts[0],
                                    0.2);

    //std::cout << "Threshold: " << threshold << " numc:" << numc << " fnumc:" << fnumc << std::endl;
    //RW_DEBUGS("Threshold: " << threshold << " numc:" << numc << " fnumc:" << fnumc);
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
        OBRManifold manifold(0.02,1.2);
        //std::cout << "Adding clustered points to manifold!" << std::endl;
        for(;idxFrom<idxTo; idxFrom++){
            ContactPoint &point = src[_dstIdx[idxFrom]];
            //std::cout << point.p << std::endl;

            if( !manifold.addPoint(point) ){
                // hmm, create a new manifold for this point
            }
        }
        manifolds.push_back(manifold);
    }

    // run through all manifolds and get the contact points that will be used.
    int contactIdx = 0;
    rw::math::Vector3D<> contactNormalAvg(0,0,0);
    BOOST_FOREACH(OBRManifold& obr, manifolds){
        contactNormalAvg += obr.getNormal();
        int nrContacts = obr.getNrOfContacts();
        for(int j=0;j<nrContacts; j++){
            _allcontacts.push_back( obr.getContact(j) );
            dst[contactIdx] = obr.getContact(j);
            contactIdx++;
        }
    }

    // get the friction data and contact settings
    dContact conSettings;
    _odeMaterialMap->setContactProperties(conSettings, dataB1, dataB2);

    // TODO: collisionMargin should also be specified per object
    //double collisionMargin = _dwc->getCollisionMargin();

    RW_DEBUGS("- Filtered contacts: " << numc << " --> " << fnumc);
    //std::cout << "- Filtered contacts: " << numc << " --> " << fnumc << std::endl;
    Frame *world = _dwc->getWorkcell()->getWorldFrame();
    _nrOfCon = fnumc;
    std::vector<dJointFeedback*> feedbacks;
    std::vector<dContactGeom> feedbackContacts;
    bool enableFeedback = false;
    if(odeSensorb1!=NULL || odeSensorb2!=NULL){
        if( (dataB1->getType()!=ODEBody::FIXED) && (dataB2->getType()!=ODEBody::FIXED) ){
            if( (f1 != world) && (f2!=world) ){
                enableFeedback = true;
                //std::cout << "- detected: " << frameB1->getName() << " " << frameB2->getName() << std::endl;
            }
        }
    }
    Vector3D<> cNormal(0,0,0);
    double maxPenetration = 0;
    // Run through all contacts and define contact constraints between them
    //std::vector<ContactPoint> &contactPointList = _rwClusteredContacts; int num = numc;
    std::vector<ContactPoint> &contactPointList = dst; int num = contactIdx;
    for (int i = 0; i < num; i++) {
        ContactPoint *point = &contactPointList[i];
        point->n = normalize(point->n);
        dContact &con = *((dContact*)point->userdata);

        //std::cout << point->penetration << " " << point->n << "  " << point->p << "  " << std::endl;
        cNormal += point->n;
        double rwnlength = MetricUtil::norm2(point->n);
        if((0.9>rwnlength) || (rwnlength>1.1)){
        	std::cout <<  "\n\n Normal not normalized _0_ !\n"<<std::endl;
        	continue;
        }
        ODEUtil::toODEVector(point->n, con.geom.normal);
        ODEUtil::toODEVector(point->p, con.geom.pos);

        double odenlength = sqrt( con.geom.normal[0]*con.geom.normal[0] +
                                  con.geom.normal[1]*con.geom.normal[1] +
                                  con.geom.normal[2]*con.geom.normal[2] );


        if( (0.9>odenlength) || (odenlength>1.1) )
        	std::cout <<  "\n\n Normal not normalized _1_ !\n"<<std::endl;

        con.geom.depth = point->penetration;
        maxPenetration = std::max(point->penetration, maxPenetration);
        _maxPenetration = std::max(point->penetration, _maxPenetration);

        if( con.geom.depth <= 0){
            continue;
        }

        RW_DEBUGS("-- Depth/Pos  : " << con.geom.depth << " " << printArray(con.geom.pos,3));
        //RW_DEBUGS("-- Depth/Pos p: " << point.penetration << " " << point.p );

        // currently we use the best possible ode approximation to Coloumb friction
        con.surface = conSettings.surface;

        RW_DEBUGS("-- create contact ");
        dJointID c = dJointCreateContact (_worldId,_contactGroupId,&con);
        RW_DEBUGS("-- attach bodies");
        dJointAttach (c, b1, b2 );
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
    if(enableFeedback && odeSensorb1){
        odeSensorb1->addFeedback(feedbacks, feedbackContacts, dataB2->getRwBody(), 0);
        //odeSensorb1->setContacts(result,wTa,wTb);
    }
    if(enableFeedback && odeSensorb2){
        odeSensorb2->addFeedback(feedbacks, feedbackContacts, dataB1->getRwBody(), 1);
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


void ODESimulator::resetScene(rw::kinematics::State& state)
{
	isInErrorGlobal = false;
	_time = 0.0;

	// first run through all rigid bodies and set the velocity and force to zero
	RW_DEBUGS("- Resetting bodies: " << _bodies.size());
	BOOST_FOREACH(dBodyID body, _allbodies){
	    dBodySetLinearVel  (body, 0, 0, 0);
	    dBodySetAngularVel (body, 0, 0, 0);
	    dBodySetForce  (body, 0, 0, 0);
	    dBodySetTorque (body, 0, 0, 0);
	    Frame *frame = _rwODEBodyToFrame[body];
	    if(!frame)
	        RW_THROW("SHOULD NOT BE NULL!");
	    ODEUtil::setODEBodyT3D(body, Kinematics::worldTframe(frame, state) );
	}
    for(size_t i=0; i<_odeBodies.size(); i++){
        _odeBodies[i]->reset(state);
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
	//std::cout << "Finished reset!!" << std::endl;
}

void ODESimulator::exitPhysics()
{

}
