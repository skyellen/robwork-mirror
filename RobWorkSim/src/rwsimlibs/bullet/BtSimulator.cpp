// changles to BtSimulator class made by simat (jan 2015) is fitted to bullet-2.82

#include "BtSimulator.hpp"
#include "BtDebugRender.hpp"

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include <rw/models/JointDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

#include <rwsim/contacts/ContactDetector.hpp>

#include <boost/foreach.hpp>

using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::models;

using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::drawable;
using namespace rwsim::simulator;
using namespace rwlibs::simulation;

// define the maximum number of objects
#define MAX_PROXIES (1024)
#define COL_MARGIN 0.001

// TODO: simat - fix this terrible hack to be able to have actual friction- and restitution-values for object specific collisions
#define commonFriction 1.05;
#define commonRestitution 0.01;

namespace {

	btVector3 makeBtVector(const Vector3D<>& v3d){
		return btVector3(v3d(0),v3d(1),v3d(2));
	}

	Vector3D<> toVector3D(const btVector3& v){
        return Vector3D<>(v[0],v[1],v[2]);
    }

	btTransform makeBtTransform(const Transform3D<> &t3d){
		btTransform btt3d;
		Quaternion<> quat(t3d.R());

		btVector3 btPos(t3d.P()[0],t3d.P()[1],t3d.P()[2]);
		btQuaternion btRot(quat.getQx(),quat.getQy(),quat.getQz(),quat.getQw());

		btt3d.setOrigin(btPos);
	    btt3d.setRotation(btRot);
	    return btt3d;
	}

	btCollisionShape* createColShape(Geometry::Ptr geometry, State state, double margin){
	    TriMesh* mesh = dynamic_cast<TriMesh*>(geometry->getGeometryData().get());
	    if( mesh==NULL ){
	        return NULL;
	    }

	    btTriangleMesh* trimesh = new btTriangleMesh();

	    Transform3D<> rw_pTf = geometry->getTransform();

	    btTransform pTf = makeBtTransform( rw_pTf );

	    // TODO: remember to transform any geometry reference to root nodes reference
	    for (size_t i=0; i<mesh->getSize(); i++)
	    {
	        Triangle<> tri = mesh->getTriangle(i);
	        btVector3 v1(tri[0][0], tri[0][1], tri[0][2]);
	        btVector3 v2(tri[1][0], tri[1][1], tri[1][2]);
	        btVector3 v3(tri[2][0], tri[2][1], tri[2][2]);
	        v1 = pTf * v1;
	        v2 = pTf * v2;
	        v3 = pTf * v3;

	        trimesh->addTriangle(v1, v2, v3);
	    }
	    if (trimesh->getNumTriangles() == 0) {
	        delete trimesh;
	        return NULL;
	    }
	    // create the collision shape from the trimesh data
	    btGImpactMeshShape *colShape = new btGImpactMeshShape(trimesh);
	    colShape->setMargin(margin);

	    colShape->postUpdate();
	    colShape->updateBound();// Call this method once before doing collisions

	    return colShape;
	}


	btCollisionShape* getColShape( Geometry::Ptr geometry,
        const State &state,
        double margin)
    {
	    btTransform btTrans;
	    btTrans.setIdentity();

	    return createColShape( geometry , state, margin);
	}

	void addKinematicDevice(){

	}


}

typedef std::pair<Frame*,btRigidBody*> FrameBodyPair;

class btPositionDevice: public BtSimulator::btDevice {

public:
    btPositionDevice(KinematicDevice::Ptr dev,
                     std::vector<FrameBodyPair > frameToBtBody):
        _kdev(dev),
        _frameToBtBody(frameToBtBody)
    {
    }

    virtual ~btPositionDevice(){};

    void update(double dt, State& state){
        _kdev->getModel().setQ( _kdev->getQ(state), state);
        // for each joint update the position of the corresponding btRigidBody

        BOOST_FOREACH( FrameBodyPair& pair, _frameToBtBody ){
            Transform3D<> t3d = Kinematics::worldTframe( pair.first, state);
            pair.second->getMotionState()->setWorldTransform( makeBtTransform(t3d) );
        }
    }

    void postUpdate(State& state){

    }

private:
    KinematicDevice::Ptr _kdev;
    std::vector<std::pair<Frame*,btRigidBody*> > _frameToBtBody;
};

class btVelocityDevice: public BtSimulator::btDevice {
public:

    btVelocityDevice(RigidDevice::Ptr rdev,std::vector<btTypedConstraint*> constraints):
        _rdev(rdev),
        _constraints(constraints)
    {

    }

    virtual ~btVelocityDevice(){};

    void update(double dt, State& state){
        Q velQ = _rdev->getVelocity(state);
        for(unsigned int i = 0; i<_constraints.size(); i++){
            double vel = velQ[i];
            if(dynamic_cast<btHingeConstraint*>(_constraints[i])){
                btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(_constraints[i]);
                hinge->enableAngularMotor(true, vel, 20);
            } else if( dynamic_cast<btSliderConstraint*>(_constraints[i]) ){
                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(_constraints[i]);
                slider->setTargetLinMotorVelocity(vel);
            }

        }
    }

    void postUpdate(State& state){
        Q q = _rdev->getModel().getQ(state);
        for(unsigned int i = 0; i<_constraints.size(); i++){
            if(dynamic_cast<btHingeConstraint*>(_constraints[i])){
                btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(_constraints[i]);
                q[i] = hinge->getHingeAngle();
            } else if( dynamic_cast<btSliderConstraint*>(_constraints[i]) ){
//                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(_constraints[i]); // simat - not used so commented to suppress warning
            }
        }
        _rdev->getModel().setQ(q, state);
    }

private:
    RigidDevice::Ptr _rdev;
    std::vector<btTypedConstraint*> _constraints;

};

BtSimulator::BtSimulator():
	m_dynamicsWorld(NULL),
	m_overlappingPairCache(NULL),
	m_dispatcher(NULL),
	m_solver(NULL),
	m_collisionConfiguration(NULL),
	_dwc(NULL),
	_render( rw::common::ownedPtr(new BtDebugRender(this)) ),
	_time(0.0),
	_dt(0.001),
	_initPhysicsHasBeenRun(false)
{
};


BtSimulator::BtSimulator(DynamicWorkCell* dwc):
	m_dynamicsWorld(NULL),
	m_overlappingPairCache(NULL),
	m_dispatcher(NULL),
	m_solver(NULL),
	m_collisionConfiguration(NULL),
	_dwc(dwc),
	_render( rw::common::ownedPtr(new BtDebugRender(this)) ),
	_time(0.0),
	_dt(0.001),
	_initPhysicsHasBeenRun(false)
{
}

SimulatorDebugRender::Ptr BtSimulator::createDebugRender(){
	return NULL;
	//    return _render;
}

void BtSimulator::step(double dt, State& state){

	// update KINEMATICS OBJECT frames, links are moddeled this way
	// these need to be set before we make the update

    //std::cout << "Controller" << std::endl;
    BOOST_FOREACH(SimulatedController::Ptr controller, _controllers ){
        controller->update(dt, state);

    }
    //std::cout << "Dev" << std::endl;
    BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->update(dt,state);
    }
    //std::cout << "Step" << std::endl;
	// update all device force/velocity input
    // then for kinematic bodies

	for(size_t i=0; i<_btLinks.size(); i++){
		Transform3D<> t3d = Kinematics::worldTframe( _rwLinks[i]->getBodyFrame(), state);
		Vector3D<> posDisplacement(_rwLinks[i]->getLinVel(state)[0]*dt,_rwLinks[i]->getLinVel(state)[1]*dt,_rwLinks[i]->getLinVel(state)[2]*dt);
		Rotation3D<> rotDisplacement( RPY<>( _rwLinks[i]->getAngVel(state)[0]*dt, _rwLinks[i]->getAngVel(state)[1]*dt, _rwLinks[i]->getAngVel(state)[2]*dt).toRotation3D());
		t3d = t3d * Transform3D<> (posDisplacement,rotDisplacement);
		_btLinks[i]->getMotionState()->setWorldTransform( makeBtTransform(t3d) );
	}

	///step the simulation
	if (m_dynamicsWorld)
	{
//		m_dynamicsWorld->stepSimulation(dt,100,1.0/500.0);
	    m_dynamicsWorld->stepSimulation(dt,0);
		_time += dt;
	}


    BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->postUpdate(state);
    }

	// update the kinematic bodies of robwork to match the motion handled by bullet
	for(size_t i=0; i<_btLinks.size(); i++){
		KinematicBody::Ptr b = _rwLinks[i];
		MovableFrame &mframe = *(b->getMovableFrame());
		btRigidBody *btb = _btLinks[i];
		const btVector3 &v = btb->getCenterOfMassTransform().getOrigin();
		const btQuaternion &q = btb->getCenterOfMassTransform().getRotation();

		Vector3D<> ang = toVector3D( btb->getAngularVelocity() );
        Vector3D<> lin = toVector3D( btb->getLinearVelocity() );

		Vector3D<> pos(v[0],v[1],v[2]);
		Quaternion<> quat(q[0],q[1],q[2],q[3]);
		Transform3D<> wTp = Kinematics::worldTframe(mframe.getParent(state), state);
		mframe.setTransform( inverse(wTp) * Transform3D<>(pos,quat), state );

        b->setAngVel( ang , state);
        b->setLinVel( lin , state);
	}

	// now copy all transforms into state
	for(size_t i=0; i<_btBodies.size(); i++){
		RigidBody::Ptr b = _rwBodies[i];
		MovableFrame &mframe = *(b->getMovableFrame());
		btRigidBody *btb = _btBodies[i];
		const btVector3 &v = btb->getCenterOfMassTransform().getOrigin();
		const btQuaternion &q = btb->getCenterOfMassTransform().getRotation();

		Vector3D<> ang = toVector3D( btb->getAngularVelocity() );
        Vector3D<> lin = toVector3D( btb->getLinearVelocity() );
		Vector3D<> pos(v[0],v[1],v[2]);
		Quaternion<> quat(q[0],q[1],q[2],q[3]);
		Transform3D<> wTp = Kinematics::worldTframe(mframe.getParent(state), state);
		mframe.setTransform( inverse(wTp) * Transform3D<>(pos,quat), state );


        b->setAngVel( ang , state);
        b->setLinVel( lin , state);
	}
}

btRigidBody* BtSimulator::createRigidBody(Geometry::Ptr geometry,
                                          double mass,
                                          const State& state,
                                          double margin){
    btCollisionShape* colShape =
        getColShape(geometry, state, margin);
    if( colShape==NULL)
        return NULL;
    // if the colShape already is in the m_collisionShapes then don't add it
    bool isInList = false;
    for(int m=0;m<m_collisionShapes.size();m++){
        if( m_collisionShapes[m]==colShape )
            isInList = true;
    }
    if( !isInList ){
//        std::cout << "Col shape added!!!" << std::endl;
        m_collisionShapes.push_back(colShape);
    }

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    Transform3D<> t3d = geometry->getFrame()->getTransform(state);
    btTransform startTransform = makeBtTransform(t3d);

    btVector3 localInertia(1,1,1);
    if (isDynamic)
        colShape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
// TODO: should damping be enabled at what should the value be?
//    rbInfo.m_additionalDamping = true;
//    rbInfo.m_additionalDampingFactor = 1.5;

// TODO: simat - fix friction
// bullets set "friction" on objects and as default when bodies collidied the 2 m_friction values are multiplied together. This is not correct and is currently overridden in the CustomMaterialCombinerCallback()
    rbInfo.m_friction = 0.5;
    rbInfo.m_restitution = 0.01;

    btRigidBody* btbody = new btRigidBody(rbInfo);
    m_dynamicsWorld->addRigidBody(btbody);

    // add rigid body to to map

    _rwBtBodyToFrame[btbody] = geometry->getFrame();
    if( isDynamic ){
        _rwFrameToBtBody[geometry->getFrame()] = btbody;
    }
    return btbody;
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
    // Apply material properties
	cp.m_combinedFriction = commonFriction;//colObj0Wrap->getCollisionObject()->getFriction();
	cp.m_combinedRestitution = commonRestitution;//colObj0Wrap->getCollisionObject()->getRestitution();

    //FROM BULLET documentation: this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
    return true;
}

// simat - needed to register the CustomMaterialCombinerCallback to bullet
extern ContactAddedCallback		gContactAddedCallback;

bool BtSimulator::setContactDetector(ContactDetector::Ptr detector) {
	return false;
}

void BtSimulator::initPhysics(State& state)
{
	// simat - needed to register the CustomMaterialCombinerCallback to bullet
    gContactAddedCallback = CustomMaterialCombinerCallback;


    int nrFixedBody(0),nrRigidBody(0),nrLinkBody(0);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_dispatcher->setNearCallback(MyNearCallback);

	btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

	///the maximum size of the collision world. Make sure objects stay within these boundaries
	///Don't make the world AABB size too large, it will harm simulation quality and performance
	btVector3 worldAabbMin(-20,-20,-20);
	btVector3 worldAabbMax(20,20,20);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,MAX_PROXIES);

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;

	m_dynamicsWorld->setGravity(btVector3(0,0,-10));

	std::vector<RigidBody::Ptr> rbodies = _dwc->findBodies<RigidBody>();

	for(size_t i=0; i<rbodies.size(); i++){
		std::string fname = rbodies[i]->getBodyFrame()->getName();
//		std::cout << "Body: " << fname << std::endl;
//		std::cout << "RigidBody: " << rbodies[i]->getBodyFrame()->getName() << std::endl;
		// create a triangle mesh for all staticly connected nodes
		btRigidBody *btbody = createRigidBody(rbodies[i]->getGeometry()[0],rbodies[i]->getMass(), state, COL_MARGIN);
		btbody->setCollisionFlags(btbody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);// flag set to enable the CustomMaterialCombinerCallback to handle friction- and restitution coefficients

		_btBodies.push_back(btbody);
		_rwBodies.push_back(rbodies[i]);
		nrRigidBody++;
	}

	std::vector<FixedBody::Ptr> fbodies = _dwc->findBodies<FixedBody>();
	for(size_t i=0; i<fbodies.size(); i++){
		std::string fname = fbodies[i]->getBodyFrame()->getName();
//		std::cout << "Body: " << fname << std::endl;

//		std::cout << "FixedBody: " << fbodies[i]->getBodyFrame()->getName() << std::endl;

		btRigidBody *btbody = createRigidBody(fbodies[i]->getGeometry()[0],0.f,state,COL_MARGIN);
		btbody->setCollisionFlags( btbody->getCollisionFlags() |
								   btCollisionObject::CF_STATIC_OBJECT );
		btbody->setCollisionFlags(btbody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK); // flag set to enable the CustomMaterialCombinerCallback to handle friction- and restitution coefficients

		nrFixedBody++;
	}

	std::vector<KinematicBody::Ptr> kbodies = _dwc->findBodies<KinematicBody>();
	for(size_t i=0; i<kbodies.size(); i++){
			// use kinematic objects for visualising
//			std::cout << "KinematicBody: " << kbodies[i]->getBodyFrame()->getName() << std::endl;

            btRigidBody *btbody = createRigidBody(kbodies[i]->getGeometry()[0], 0.f, state, COL_MARGIN);
            btbody->setCollisionFlags( btbody->getCollisionFlags() |
                                       btCollisionObject::CF_KINEMATIC_OBJECT );
            btbody->setActivationState(DISABLE_DEACTIVATION);
    		btbody->setCollisionFlags(btbody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);// flag set to enable the CustomMaterialCombinerCallback to handle friction- and restitution coefficients

            _btLinks.push_back(btbody);
            _rwLinks.push_back(kbodies[i]);
			nrLinkBody++;
	}

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

	resetScene(state);

	_initPhysicsHasBeenRun=true;

//    std::cout << "BtSimulator: initializing physics" << std::endl
//              << " - Collision Margin: " << _dwc->getCollisionMargin() << std::endl
//              << " - Nr of fixed bodies: " << nrFixedBody << std::endl
//              << " - Nr of rigid bodies: " << nrRigidBody << std::endl
//              << " - Nr of controlled bodies: " << nrLinkBody << std::endl;

}

void BtSimulator::resetScene(State& state)
{
	int numObjects = 0;
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1.f/60.f,0);
		numObjects = m_dynamicsWorld->getNumCollisionObjects();
	}

	for (int i=0;i<numObjects;i++)
	{
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
				colObj->setWorldTransform( myMotionState->m_graphicsWorldTrans );
				colObj->setInterpolationWorldTransform( myMotionState->m_startWorldTrans );
				colObj->activate();
			}
			//removed cached contact points
			m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(colObj->getBroadphaseHandle(),m_dispatcher);

			//btRigidBody* body = btRigidBody::upcast(colObj);
			if (body && !body->isStaticObject())
			{
			    body->setLinearVelocity(btVector3(0,0,0));
			    body->setAngularVelocity(btVector3(0,0,0));
			}
		}
	}

    // now set the new position of all bodies
    for(size_t i=0; i<_btBodies.size(); i++){
        RigidBody::Ptr b = _rwBodies[i];
        MovableFrame &mframe = *(b->getMovableFrame());
        Transform3D<> wTp = Kinematics::worldTframe(mframe.getParent(state), state);
        Transform3D<> t3d = wTp * mframe.getTransform( state );

        btTransform bttrans = makeBtTransform(t3d);

        btRigidBody *btb = _btBodies[i];
        btb->setCenterOfMassTransform(bttrans);
    }


    //std::cout << "Dev" << std::endl;
    /*BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->update(0,state);
    }*/

}

void BtSimulator::exitPhysics(){
	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	if(_initPhysicsHasBeenRun){
		int i;
	   //removed/delete constraints
		for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
		{
			btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
			m_dynamicsWorld->removeConstraint(constraint);
			delete constraint;
		}

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
		for (int j=0;j<m_collisionShapes.size();j++){
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}
		//delete dynamics world
		delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

//delete broadphase
	delete m_overlappingPairCache;

//delete dispatcher

	delete m_dispatcher;

	delete m_collisionConfiguration;
	}

}

btDynamicsWorld* BtSimulator::getBtWorld() const {
    return m_dynamicsWorld;
}

void BtSimulator::load(DynamicWorkCell::Ptr dwc){
	_dwc = dwc;
	_materialMap = dwc->getMaterialData();
	_contactMap = dwc->getContactData();
}

// simat - functions below is not used but were essential for succesful compile
void BtSimulator::addDevice(DynamicDevice::Ptr dev, State& nstate){
	std::cout << "DANGER: called not finished function: a12" << std::endl;

	// TODO: implement method
}

void BtSimulator::attach(Body::Ptr b1, Body::Ptr b2){
	std::cout << "DANGER: called not finished function: a11" << std::endl;

	 // TODO: implement method
}

void BtSimulator::detach(Body::Ptr b1, Body::Ptr b2){
	std::cout << "DANGER: called not finished function: a10" << std::endl;

	 // TODO: implement method
}

void BtSimulator::addSensor(SimulatedSensor::Ptr sensor, State& state){
	std::cout << "DANGER: called not finished function: a9" << std::endl;

	 // TODO: implement method
}

void BtSimulator::setDynamicsEnabled(Body::Ptr body, bool enabled){
	std::cout << "DANGER: called not finished function: a9" << std::endl;

	 // TODO: implement method
}

void BtSimulator::DWCChangedListener(DynamicWorkCell::DWCEventType type, boost::any data){
	std::cout << "DANGER: called not finished function: a8" << std::endl;

	 // TODO: implement method
}

void BtSimulator::setEnabled(Body::Ptr body, bool enabled){
	std::cout << "DANGER: called not finished function: a7" << std::endl;

	 // TODO: implement method
}

void BtSimulator::emitPropertyChanged(){
	std::cout << "DANGER: called not finished function: a6" << std::endl;

	 // TODO: implement method
}

void BtSimulator::addBody(Body::Ptr body, State& state){
	std::cout << "DANGER: called not finished function: a5" << std::endl;

	 // TODO: implement method
}

void BtSimulator::addConstraint(Constraint::Ptr constraint) {
	std::cout << "DANGER: called not finished function: a4" << std::endl;

	 // TODO: implement method
}

void BtSimulator::removeSensor(SimulatedSensor::Ptr sensor){
	std::cout << "DANGER: called not finished function: a3" << std::endl;

	 // TODO: implement method
}

void BtSimulator::disableCollision(Body::Ptr b1, Body::Ptr b2){
	std::cout << "DANGER: called not finished function: a2" << std::endl;

	 // TODO: implement method
}

void BtSimulator::enableCollision(Body::Ptr b1, Body::Ptr b2){
	std::cout << "DANGER: called not finished function: a1" << std::endl;
	 // TODO: implement method
}
