/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtSimulator.hpp"
#include "BtDebugRender.hpp"
#include "BtBody.hpp"
#include "BtUtil.hpp"
#include "BtConstraint.hpp"
#include "BtTactileSensor.hpp"
#include "BtDevice.hpp"
#include "BtMaterial.hpp"
#include "BtContactStrategy.hpp"
#include "BtRWCollisionConfiguration.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>

#if 0
#include <rw/models/JointDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

using namespace rw::math;
using namespace rw::models;
#endif

#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::kinematics;

using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::drawable;
using namespace rwsim::sensor;
using namespace rwlibs::simulation;
using namespace rwsimlibs::bullet;

// define the maximum number of objects
#define MAX_PROXIES (1024)

BtSimulator::BtSimulator():
	m_dynamicsWorld(NULL),
	m_overlappingPairCache(NULL),
	m_dispatcher(NULL),
	m_solver(NULL),
	m_collisionConfiguration(NULL),
	_render( rw::common::ownedPtr(new BtDebugRender(this)) ),
	_time(0.0),
	_dt(0.001),
	_initPhysicsHasBeenRun(false)
{
}

BtSimulator::BtSimulator(DynamicWorkCell::Ptr dwc):
	m_dynamicsWorld(NULL),
	m_overlappingPairCache(NULL),
	m_dispatcher(NULL),
	m_solver(NULL),
	m_collisionConfiguration(NULL),
	_render( rw::common::ownedPtr(new BtDebugRender(this)) ),
	_time(0.0),
	_dt(0.001),
	_initPhysicsHasBeenRun(false)
{
	load(dwc);
}

BtSimulator::~BtSimulator()	{
	exitPhysics();
}

void BtSimulator::load(DynamicWorkCell::Ptr dwc){
	_dwc = dwc;
	_materialMap = dwc->getMaterialData();
	_contactMap = dwc->getContactData();

	_detector = ownedPtr(new ContactDetector(_dwc->getWorkcell()));
	_detectorData = ownedPtr(new ContactDetectorData());
	const ContactStrategy::Ptr strat = ownedPtr(new BtContactStrategy());
	strat->setPropertyMap(_dwc->getEngineSettings());
	_detector->addContactStrategy(strat);
}

bool BtSimulator::setContactDetector(ContactDetector::Ptr detector) {
	_detector = detector;
	_detectorData = ownedPtr(new ContactDetectorData());
	return true;
}

void BtSimulator::step(double dt, State& state){

	// update KINEMATICS OBJECT frames, links are moddeled this way
	// these need to be set before we make the update

    //std::cout << "Controller" << std::endl;
	Simulator::UpdateInfo info(dt);
	info.dt_prev = dt;
    BOOST_FOREACH(SimulatedController::Ptr controller, _controllers ) {
        controller->update(info, state);

    }
    //std::cout << "Dev" << std::endl;
    BOOST_FOREACH(BtDevice *dev, _btDevices){
        dev->update(dt,state);
    }

    //std::cout << "Step" << std::endl;
	// update all device force/velocity input
    // then for kinematic bodies

    BOOST_FOREACH(BtBody* const body, _btBodies) {
    	body->update(dt, state); // updates velocities for kinematic objects
    }

    BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
    	constraint->update(dt, state); // apply spring forces
    }

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(dt,1,dt); // Fixed time-stepping
		_time += dt;
	}

	for (int i = 0; i < m_dynamicsWorld->getDispatcher()->getNumManifolds(); i++) {
		const btPersistentManifold* const manifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* const objectA = manifold->getBody0();
		const btCollisionObject* const objectB = manifold->getBody1();
		const BtBody::BodyMetaData* const dataA = static_cast<BtBody::BodyMetaData*>(objectA->getUserPointer());
		const BtBody::BodyMetaData* const dataB = static_cast<BtBody::BodyMetaData*>(objectB->getUserPointer());
		const BtBody* const bodyA = _rwFrameToBtBody[dataA->frame];
		const BtBody* const bodyB = _rwFrameToBtBody[dataB->frame];
	    BOOST_FOREACH(BtTactileSensor* const sensor, _btSensors) {
	    	sensor->addContactManifold(info, state, manifold, bodyA, bodyB);
	    }
	}

    BOOST_FOREACH(BtTactileSensor* const sensor, _btSensors) {
    	sensor->addConstraintsFeedback(info, state);
    }

    BOOST_FOREACH(BtDevice *dev, _btDevices){
        dev->postUpdate(state);
    }

    BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
    	constraint->postUpdate(state);
    }

	// now copy all transforms into state
    BOOST_FOREACH(BtBody* const body, _btBodies) {
		body->postupdate(state);
	}
}

void BtSimulator::resetScene(State& state)
{
	if(!_initPhysicsHasBeenRun) {
		RW_THROW("BtSimulator (resetScene): initPhysics has not been run!");
	}

	/*int numObjects = 0;
	if (m_dynamicsWorld) {
		m_dynamicsWorld->stepSimulation(1.f/60.f,0);
		numObjects = m_dynamicsWorld->getNumCollisionObjects();
	}

	for (int i=0;i<numObjects;i++) {
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body) {
			if (body->getMotionState())	{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
				colObj->setWorldTransform( myMotionState->m_graphicsWorldTrans );
				colObj->setInterpolationWorldTransform( myMotionState->m_startWorldTrans );
				colObj->activate();
			}
			//removed cached contact points
			m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(colObj->getBroadphaseHandle(),m_dispatcher);

			//btRigidBody* body = btRigidBody::upcast(colObj);
			if (body && !body->isStaticObject()) {
			    body->setLinearVelocity(btVector3(0,0,0));
			    body->setAngularVelocity(btVector3(0,0,0));
			}
		}
	}*/

	// Delete all sensors
	BOOST_FOREACH(BtTactileSensor* const sensor, _btSensors) {
		delete sensor;
	}
	_btSensors.clear();
	_sensors.clear();

	// Delete all constraints (sensors must be deleted first)
	BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
		delete constraint;
	}
	_constraints.clear();

	// Delete all bodies (constraints must be deleted first)
	BOOST_FOREACH(BtBody* const btBody, _btBodies) {
		delete btBody;
	}
	_btBodies.clear();
	_rwFrameToBtBody.clear();
	_rwBtBodyToFrame.clear();

	// Now add new bodies with new initial state
	BOOST_FOREACH(const Body::Ptr body, _dwc->getBodies()) {
		addBody(body,state);
	}

	// Add the constraints
	const DynamicWorkCell::ConstraintList& constraints = _dwc->getConstraints();
	BOOST_FOREACH(const Constraint::Ptr constraint, constraints) {
		addConstraint(constraint);
	}

	// Add the sensors
	const DynamicWorkCell::SensorList& sensors = _dwc->getSensors();
	BOOST_FOREACH(const SimulatedSensor::Ptr sensor, sensors) {
		addSensor(sensor, state);
	}

    //std::cout << "Dev" << std::endl;
    /*BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->update(0,state);
    }*/

}

//typedef void (*btNearCallback)(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);

void MyNearCallback(btBroadphasePair& collisionPair,
                    btCollisionDispatcher& dispatcher,
                    const btDispatcherInfo& dispatchInfo)
{
    // Do your collision logic here
    // Only dispatch the bullet collision information if you want the physics to continue
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

// TODO: simat - fix this bad friction fix to allow for multiobject individual friction and restitution
// current problem is that from this function there is no access to object material on the btCollisionObjectWrapper passed to this function
static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1){
	const BtBody::BodyMetaData* const dataA = static_cast<BtBody::BodyMetaData*>(colObj0Wrap->getCollisionObject()->getUserPointer());
	const BtBody::BodyMetaData* const dataB = static_cast<BtBody::BodyMetaData*>(colObj1Wrap->getCollisionObject()->getUserPointer());
	const BtMaterial* const matA = dataA->material;
	const BtMaterial* const matB = dataB->material;
	if (!matA || !matB)
		RW_THROW("Material for one or both of the bodies was not set!");
	// Apply material properties
	cp.m_combinedFriction = BtMaterial::getFriction(matA,matB);
	cp.m_combinedRestitution = BtMaterial::getRestitution(matA,matB);

    //FROM BULLET documentation: this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
    return true;
}

// simat - needed to register the CustomMaterialCombinerCallback to bullet
extern ContactAddedCallback		gContactAddedCallback;

void BtSimulator::initPhysics(State& state) {
	if (_dwc.isNull())
		RW_THROW("BtSimulator could not initialize physics as the dynamic workcell was null!");
	_propertyMap = _dwc->getEngineSettings();

	// simat - needed to register the CustomMaterialCombinerCallback to bullet
    gContactAddedCallback = CustomMaterialCombinerCallback;

	///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new BtRWCollisionConfiguration(_detector);

	///use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_dispatcher->setNearCallback(MyNearCallback);

	//btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

	///the maximum size of the collision world. Make sure objects stay within these boundaries
	///Don't make the world AABB size too large, it will harm simulation quality and performance
	btVector3 worldAabbMin(-20,-20,-20);
	btVector3 worldAabbMax(20,20,20);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,MAX_PROXIES);

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);

	const PropertyMap& propertyMap = _propertyMap;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true; // Default true
	m_dynamicsWorld->getSolverInfo().m_erp = propertyMap.get<double>("WorldERP", 0.2); // Default 0.2 (Baumgarte - used as default for constraints)
	m_dynamicsWorld->getSolverInfo().m_erp2 = propertyMap.get<double>("WorldERP", 0.8); // Default 0.8 (split impulse - only for contacts?)
	m_dynamicsWorld->getSolverInfo().m_globalCfm = propertyMap.get<double>("WorldCFM", 0); // Default 0.

	m_dynamicsWorld->setGravity(BtUtil::makeBtVector(_dwc->getGravity()));

#if 0
	// ok, now add constraints
	BOOST_FOREACH(DynamicDevice::Ptr device, _dwc->getDynamicDevices() ){
		if( device.cast<RigidDevice>()!=NULL ){
			std::cout << "RigidDevice...." << std::endl;
			// add kinematic constraints from base to joint1, joint1 to joint2 and so forth
			RigidDevice::Ptr fDev = device.cast<RigidDevice>();
	        JointDevice *jdev = dynamic_cast<JointDevice*>( &(fDev->getModel()) );
			std::vector<btTypedConstraint*> constraints;
			std::cout << "Nr of Joints " << jdev->getDOF()<< std::endl;

			// handle base of robot specificly
			/*Frame *base = jdev->getBase();
	        if( _rwFrameToBtBody[base]==NULL ){
	            // base is not staticly connected to some
	            std::cout << "Base is not connected!!" << std::endl;
	            createRigidBody(base, 0.2 , state, 0.001 );

	            btRigidBody *btParent = _rwFrameToBtBody[ base->getParent(state) ];

	            // add static constraint between base and its parent
	        }

	        btRigidBody *btBase = _rwFrameToBtBody[base];
			 */

			for(size_t i=0;i<jdev->getDOF();i++){

				Joint *joint = jdev->getJoints()[i];
				Body::Ptr link = fDev->getLinks()[i];
				Frame *parent = joint->getParent(state);

				std::cout << parent->getName() << "-->" << joint->getName() << std::endl;

				btRigidBody *btParent = _rwFrameToBtBody[parent];
				const Frame* parentFrame = NULL;
				if(btParent==NULL){
					RW_WARN("btParent is NULL, " << joint->getName());
				} else {
					parentFrame = _rwBtBodyToFrame[btParent];
				}

				btRigidBody *btChild = createRigidBody(link->getGeometry()[0], 0.2 , state, 0.001 ); //_rwFrameToBtBody[joint];
				if(btChild==NULL){
					RW_WARN("btChild is NULL, " << joint->getName());
					RW_ASSERT(0);
				}



				std::cout << parent->getName() << " " << joint->getName() << std::endl;
				Transform3D<> wTchild = Kinematics::worldTframe(joint,state);

				Transform3D<> wTparent;
				if(parentFrame!=NULL)
					wTparent = Kinematics::worldTframe(parentFrame,state);

				btTransform frameInA, frameInB;
				frameInA.setIdentity(); // = makeBtTransform( childT3d );
				frameInB = makeBtTransform( inverse(wTparent) * wTchild );

				btChild->setActivationState(DISABLE_DEACTIVATION);
				if(btParent!=NULL)
					btParent->setActivationState(DISABLE_DEACTIVATION);

				if(dynamic_cast<RevoluteJoint*>(joint)){
					std::cout << "Hinge constraint" << std::endl;
					btHingeConstraint *hinge;
					if( btParent != NULL){
						hinge = new btHingeConstraint(*btChild, *btParent, frameInA, frameInB);
					} else {
						hinge = new btHingeConstraint(*btChild, frameInA);
					}
					hinge->setLimit( joint->getBounds().first[0], joint->getBounds().second[0]
																							//,softness, bias, relaxation
					);
					hinge->setAngularOnly(false);
					hinge->enableAngularMotor(true, 0.0, 10);
					m_dynamicsWorld->addConstraint(hinge, true);

					_jointToConstraintMap[joint] = hinge;
					constraints.push_back(hinge);
				} else if( dynamic_cast<PrismaticJoint*>(joint) ){
					std::cout << "Slider constraint" << std::endl;
					btSliderConstraint *slider = new btSliderConstraint(*btChild, *btParent, frameInA, frameInB, true);
					slider->setLowerLinLimit( joint->getBounds().first[0] );
					slider->setUpperLinLimit( joint->getBounds().second[0] );
					slider->setLowerAngLimit(-0.00001);
					slider->setUpperAngLimit( 0.00001);

					slider->setPoweredLinMotor(true);
					slider->setMaxLinMotorForce(0.1);
					slider->setTargetLinMotorVelocity(0.0);

					m_dynamicsWorld->addConstraint(slider, true);
					_jointToConstraintMap[joint] = slider;
					constraints.push_back(slider);
					//} else if(  ) {

					} else {
						RW_WARN("Joint type not supported!");
					}
			}
			_btDevices.push_back( new btVelocityDevice(fDev,constraints) );

		} else  if(device.cast<KinematicDevice>() != NULL) {
			std::cout << "KinematicDevice...." << std::endl;
			const KinematicDevice::Ptr kdev = device.cast<KinematicDevice>();
			std::vector<FrameBodyPair> frameBodyList;
			BOOST_FOREACH(const Body::Ptr body, kdev->getLinks() ){
				Frame* const frame = body->getBodyFrame();
				btRigidBody *btBody = _rwFrameToBtBody[frame];

				frameBodyList.push_back( std::make_pair(frame, btBody ) );
			}

			_btDevices.push_back( new btPositionDevice(kdev, frameBodyList) );
		} else {
			RW_WARN("Controller not supported!");
		}
		_devices.push_back(device);
	}
#endif

	_initPhysicsHasBeenRun=true;

	resetScene(state);

//    std::cout << "BtSimulator: initializing physics" << std::endl
//              << " - Collision Margin: " << _dwc->getCollisionMargin() << std::endl
//              << " - Nr of fixed bodies: " << nrFixedBody << std::endl
//              << " - Nr of rigid bodies: " << nrRigidBody << std::endl
//              << " - Nr of controlled bodies: " << nrLinkBody << std::endl;

}

void BtSimulator::exitPhysics(){
	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	if(_initPhysicsHasBeenRun){

		BOOST_FOREACH(BtTactileSensor* const sensor, _btSensors) {
			delete sensor;
		}
		_btSensors.clear();
		_sensors.clear();

		BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
			delete constraint;
		}
		_constraints.clear();

		BOOST_FOREACH(BtBody* const btBody, _btBodies) {
			delete btBody;
		}
		_btBodies.clear();

		int i;
		//removed/delete constraints
		//for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
		//{
			//btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
			//m_dynamicsWorld->removeConstraint(constraint);
			//delete constraint;
		//}

		for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--){
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()){
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}

		//delete collision shapes
		/*for (int j=0;j<m_collisionShapes.size();j++){
			const btCollisionShape* const shape = m_collisionShapes[j];
			//for (int childNo = 0; childNo < shape->get)
			delete shape;
		}*/

		//delete dynamics world
		delete m_dynamicsWorld;

		//delete solver
		delete m_solver;

		//delete broadphase
		delete m_overlappingPairCache;

		//delete dispatcher

		delete m_dispatcher;

		delete m_collisionConfiguration;
		_initPhysicsHasBeenRun = false;
	}

}

double BtSimulator::getTime() {
	return _time;
}

void BtSimulator::setEnabled(Body::Ptr body, bool enabled){
	RW_THROW("BtSimulator (setEnabled): not implemented yet!");
}

void BtSimulator::setDynamicsEnabled(Body::Ptr body, bool enabled){
	RW_THROW("BtSimulator (setDynamicsEnabled): not implemented yet!");
}

SimulatorDebugRender::Ptr BtSimulator::createDebugRender(){
	return _render;
}

PropertyMap& BtSimulator::getPropertyMap() {
	return _propertyMap;
};

void BtSimulator::emitPropertyChanged(){
	RW_THROW("BtSimulator (emitPropertyChanged): not implemented yet!");
}

void BtSimulator::addController(rwlibs::simulation::SimulatedController::Ptr controller) {
	_controllers.push_back(controller);
}

void BtSimulator::removeController(rwlibs::simulation::SimulatedController::Ptr controller) {
}

void BtSimulator::addBody(Body::Ptr body, State& state){
	BtBody* const exists = _rwFrameToBtBody[body->getBodyFrame()];
	if (!exists) {
		BtBody* const btbody = new BtBody(body, &_dwc->getMaterialData(), &_dwc->getContactData(), m_dynamicsWorld, state);
		_btBodies.push_back(btbody);
		_rwFrameToBtBody[body->getBodyFrame()] = btbody;
		_rwBtBodyToFrame[btbody] = body->getBodyFrame();
	}
}

void BtSimulator::addDevice(DynamicDevice::Ptr dev, State& nstate){
	RW_THROW("BtSimulator (addDevice): not implemented yet!");
}

void BtSimulator::addSensor(SimulatedSensor::Ptr sensor, State& state){
	if(const rw::common::Ptr<SimulatedTactileSensor> tsensor = sensor.cast<SimulatedTactileSensor>()){
		if(const rw::common::Ptr<SimulatedFTSensor> ftsensor = tsensor.cast<SimulatedFTSensor>()){
			Frame* const parentFrame = ftsensor->getBody1()->getBodyFrame();
			Frame* const sensorFrame = ftsensor->getBody2()->getBodyFrame();

			if(_rwFrameToBtBody.find(sensorFrame) == _rwFrameToBtBody.end()){
				RW_THROW("BtSimulator (addSensor): The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics?");
			}
			if( _rwFrameToBtBody.find(parentFrame)== _rwFrameToBtBody.end()){
				RW_THROW("BtSimulator (addSensor): The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics?");
			}
			BtBody* const parentBtBody = _rwFrameToBtBody[parentFrame];
			BtBody* const sensorBtBody = _rwFrameToBtBody[sensorFrame];

			BtTactileSensor* const btsensor = new BtTactileSensor(tsensor);
			_btSensors.push_back(btsensor);
			_sensors.push_back(sensor);

			// Find all constraints between the two bodies
			BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
				btRigidBody* const parent = constraint->getBtParent();
				btRigidBody* const child = constraint->getBtChild();
				if (parent == parentBtBody->getBulletBody() || parent == sensorBtBody->getBulletBody()) {
					if (child == parentBtBody->getBulletBody() || child == sensorBtBody->getBulletBody()) {
						btsensor->addFeedback(constraint);
					}
				}
			}

		} else {
			Frame* const bframe = tsensor->getFrame();
			RW_ASSERT(bframe!=NULL);

			if(_rwFrameToBtBody.find(bframe) == _rwFrameToBtBody.end()){
				RW_THROW("BtSimulator (addSensor): The frame that the sensor is being attached to is not in the simulator! Did you remember to run initphysics?");
			}
			BtBody* btBody = _rwFrameToBtBody[bframe];

			BtTactileSensor* const btsensor = new BtTactileSensor(tsensor);
			_btSensors.push_back(btsensor);
			_sensors.push_back(sensor);

			// Find all constraints between the two bodies
			BOOST_FOREACH(BtConstraint* const constraint, _constraints) {
				btRigidBody* const parent = constraint->getBtParent();
				btRigidBody* const child = constraint->getBtChild();
				if (parent == btBody->getBulletBody() || child == btBody->getBulletBody()) {
					btsensor->addFeedback(constraint);
				}
			}
		}
	} else {
		RW_THROW("BtSimulator (addSensor): the type of sensor is not supported yet!");
	}
}

void BtSimulator::removeSensor(SimulatedSensor::Ptr sensor){
	RW_THROW("BtSimulator (removeSensor): not implemented yet!");
}

void BtSimulator::attach(Body::Ptr b1, Body::Ptr b2){
	RW_THROW("BtSimulator (attach): not implemented yet!");
}

void BtSimulator::detach(Body::Ptr b1, Body::Ptr b2){
	RW_THROW("BtSimulator (detach): not implemented yet!");
}

std::vector<SimulatedSensor::Ptr> BtSimulator::getSensors() {
	return _sensors;
}

void BtSimulator::addConstraint(rw::common::Ptr<const Constraint> constraint) {
	BtBody* const parent = _rwFrameToBtBody[constraint->getBody1()->getBodyFrame()];
	BtBody* const child = _rwFrameToBtBody[constraint->getBody2()->getBodyFrame()];
	if (parent == NULL)
		RW_THROW("BtSimulator (addConstraint): could not find the parent body for the given constraint.");
	if (child == NULL)
		RW_THROW("BtSimulator (addConstraint): could not find the child body for the given constraint.");
	BtConstraint* const btconstraint = new BtConstraint(constraint, parent, child, m_dynamicsWorld);
	_constraints.push_back(btconstraint);
}
