#include "BtSimulator.hpp"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <btBulletDynamicsCommon.h>
// include gimpact stuff
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
//#include <GIMPACTUtils/btGImpactConvexDecompositionShape.h>

#include <rw/models/JointDevice.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/loaders/GeometryFactory.hpp>

#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>

#include <boost/foreach.hpp>

using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwlibs::simulation;
//using namespace rwsim::sensor;


using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::models;
using namespace rw::math;

// define the maximum number of objects
#define MAX_PROXIES (1024)
//#define COL_MARGIN 0.04

namespace {


	btVector3 makeBtVector(const rw::math::Vector3D<>& v3d){
		return btVector3(v3d(0),v3d(1),v3d(2));
	}

	rw::math::Vector3D<> toVector3D(const btVector3& v){
        return Vector3D<>(v[0],v[1],v[2]);
    }

	btTransform makeBtTransform(const rw::math::Transform3D<> &t3d){
		btTransform btt3d;
		Quaternion<> quat(t3d.R());

		btVector3 btPos(t3d.P()[0],t3d.P()[1],t3d.P()[2]);
		btQuaternion btRot(quat.getQx(),quat.getQy(),quat.getQz(),quat.getQw());

		btt3d.setOrigin(btPos);
	    btt3d.setRotation(btRot);
	    return btt3d;
	}

	typedef std::pair<CollisionModelInfo,Transform3D<> > ColInfoPair;

	btCollisionShape* createColShape(ColInfoPair &colInfo, rwsim::simulator::BtSimulator::ColCache& colCache, double margin, bool cacheEnabled){
	    std::string geofile = colInfo.first.getId();
	    Transform3D<> colT3d = colInfo.first.getTransform();
	    colInfo.second = colInfo.second*colT3d;

	    // if model lie in cache then we are finished
	    if (cacheEnabled && colCache.has(geofile) ) {
	        //std::cout << "BT CACHE HIT" << std::endl;
	        return colCache.get(geofile).get();
	    }

	    Geometry::Ptr geom = GeometryFactory::loadCollisionGeometry(colInfo.first);

	    TriMesh* mesh = dynamic_cast<TriMesh*>(geom->getGeometryData().get());
	    if( mesh==NULL ){
	        return NULL;
	    }

	    btTriangleMesh* trimesh = new btTriangleMesh();

	    Transform3D<> rw_pTf = colInfo.second;//colT3d;
	    //if(frame!=parent){
	    //    rw_pTf = Kinematics::FrameTframe(parent, frame,state)*colT3d;
	    //}
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
	    //bool useQuantizedBvhTree = true;
	    //btCollisionShape* colShape  = new btBvhTriangleMeshShape(trimesh,useQuantizedBvhTree);
	    btGImpactMeshShape *colShape = new btGImpactMeshShape(trimesh);

	    //btGImpactConvexDecompositionShape *colShape  = new
	    //    btGImpactConvexDecompositionShape(
	    //           trimesh, btVector3(1.f,1.f,1.f), btScalar(margin) );

	    colShape->setMargin(margin);

	    colShape->postUpdate();
	    colShape->updateBound();// Call this method once before doing collisions

	    if(cacheEnabled) colCache.add(geofile,colShape);
	    //colShapes.push_back(colShape);
	    return colShape;
	}


	btCollisionShape* getColShapes(
        const std::vector<Frame*>& frames,
        rw::kinematics::Frame* parent,
        BtSimulator::ColCache& colCache,
        const rw::kinematics::State &state,
        double margin)
    {
	    bool cacheEnabled = true;

	    std::vector< ColInfoPair > colModelInfos;
	    BOOST_FOREACH(const Frame* frame, frames){
            // check if frame has collision descriptor
            if (frame==NULL || CollisionModelInfo::get(frame).size()==0 )
                continue;
            Transform3D<> t3d = Kinematics::frameTframe(parent, frame,state);
            BOOST_FOREACH(CollisionModelInfo info, CollisionModelInfo::get(frame)){
                colModelInfos.push_back( ColInfoPair(info,t3d) );
            }
	    }
	    btTransform btTrans;
	    btTrans.setIdentity();
	    if( colModelInfos.size()==1 ){
	        return createColShape( colModelInfos[0] , colCache, margin , cacheEnabled);
	    } else if(colModelInfos.size()>1) {
	        // create compound shape
	        btCompoundShape *cshape = new btCompoundShape();
	        BOOST_FOREACH(ColInfoPair pair, colModelInfos){
	            btCollisionShape *shape = createColShape( pair , colCache,margin , cacheEnabled );
	            //cshape->addChildShape(makeBtTransform(pair.second), shape);
	            cshape->addChildShape(btTrans, shape);
	        }
	        return cshape;
	    }
	    return NULL;
	    /*
		std::vector<btCollisionShape*> colShapes;

		std::vector<btTransform> transform;

		// add triangles from each node
		BOOST_FOREACH(Frame* frame, frames){
			if (frame==NULL)
				continue;
			// check if frame has collision descriptor
			if ( !Accessor::collisionModelInfo().has(*frame) )
				continue;
			// get the geo descriptor
			std::string geofile = Accessor::collisionModelInfo().get(*frame)[0].getId();
			Transform3D<> colT3d = Accessor::collisionModelInfo().get(*frame)[0].getTransform();
			// if model lie in cache then we are finished
			if (cacheEnabled && colCache.has(geofile) ) {
                colShapes.push_back(colCache.get(geofile).get() );
                std::cout << "BT CACHE HIT" << std::endl;
                continue;
            }

			std::vector<Face<float> > result;
			FaceArrayFactory::loadFaceArrayFile(geofile, result);
			btTriangleMesh* trimesh = new btTriangleMesh();

			Transform3D<> rw_pTf = colT3d;
			if(frame!=parent){
			    rw_pTf = Kinematics::FrameTframe(parent, frame,state)*colT3d;
			}
            btTransform pTf = makeBtTransform( rw_pTf );

			// TODO: remember to transform any geometry reference to root nodes reference
			for (size_t i=0; i<result.size(); i++)
			{
				Face<float> &f = result[i];
				btVector3 v1(f._vertex1[0], f._vertex1[1], f._vertex1[2]);
				btVector3 v2(f._vertex2[0], f._vertex2[1], f._vertex2[2]);
				btVector3 v3(f._vertex3[0], f._vertex3[1], f._vertex3[2]);

				v1 = pTf * v1;
				v2 = pTf * v2;
				v3 = pTf * v3;

				trimesh->addTriangle(v1, v2, v3);
			}
			if (trimesh->getNumTriangles() == 0) {
                delete trimesh;
                continue;
            }
			// create the collision shape from the trimesh data
	        //bool useQuantizedBvhTree = true;
	        //btCollisionShape* colShape  = new btBvhTriangleMeshShape(trimesh,useQuantizedBvhTree);
	        btGImpactMeshShape *colShape = new btGImpactMeshShape(trimesh);

			//btGImpactConvexDecompositionShape *colShape  = new
	        //    btGImpactConvexDecompositionShape(
	        //           trimesh, btVector3(1.f,1.f,1.f), btScalar(margin) );

			colShape->setMargin(margin);

	        colShape->postUpdate();
	        colShape->updateBound();// Call this method once before doing collisions

	        if(cacheEnabled) colCache.add(geofile,colShape);
	        colShapes.push_back(colShape);
		}
		return colShapes;
		*/
	}

	btTriangleMesh* createTriMesh(
			const std::vector<Frame*>& frames,
			rw::kinematics::Frame* parent,
			rw::kinematics::State &state){
	   btTriangleMesh* trimesh = new btTriangleMesh();

	   // add triangles from each node
	   BOOST_FOREACH(Frame* frame, frames){
		   if( frame==NULL )
			   continue;
		   // check if frame has collision descriptor
		   if( CollisionModelInfo::get(frame).size()==0 )
			   continue;
		   // get the geo descriptor
		   std::string geofile = CollisionModelInfo::get(frame)[0].getId();
		   Transform3D<> colT3d = CollisionModelInfo::get(frame)[0].getTransform();

		   PlainTriMeshN1F::Ptr mesh = STLFile::load(geofile);

		   // TODO: remember to transform any geometry reference to root nodes reference
           Transform3D<> rw_pTf = colT3d;
            if(frame!=parent){
                rw_pTf = Kinematics::frameTframe(parent, frame,state)*colT3d;
            }
            btTransform pTf = makeBtTransform( rw_pTf );

		   for (size_t i=0;i<mesh->getSize();i++){
		       TriangleN1<float> &tri = (*mesh)[i];
	            btVector3 v1(tri[0][0], tri[0][1], tri[0][2]);
	            btVector3 v2(tri[1][0], tri[1][1], tri[1][2]);
	            btVector3 v3(tri[2][0], tri[2][1], tri[2][2]);

			   v1 = pTf * v1;
			   v2 = pTf * v2;
			   v3 = pTf * v3;
			   trimesh->addTriangle(v1,v2,v3);
		   }
	   }
	   if( trimesh->getNumTriangles() == 0 ){
		   delete trimesh;
		   return NULL;
	   }
	   std::cout << "TriMesh size: " << trimesh->getNumTriangles()<< std::endl;
	   return trimesh;
	}

	void addKinematicDevice(){

	}


}

typedef std::pair<Frame*,btRigidBody*> FrameBodyPair;

class btPositionDevice: public BtSimulator::btDevice {

public:
    btPositionDevice(KinematicDevice* dev,
                     std::vector<FrameBodyPair > frameToBtBody):
        _kdev(dev),
        _frameToBtBody(frameToBtBody)
    {
    }

    virtual ~btPositionDevice(){};

    void update(double dt, rw::kinematics::State& state){
        _kdev->getModel().setQ( _kdev->getQ(state), state);
        // for each joint update the position of the corresponding btRigidBody

        BOOST_FOREACH( FrameBodyPair& pair, _frameToBtBody ){
            Transform3D<> t3d = Kinematics::worldTframe( pair.first, state);
            pair.second->getMotionState()->setWorldTransform( makeBtTransform(t3d) );
        }
    }

    void postUpdate(rw::kinematics::State& state){

    }

private:
    KinematicDevice *_kdev;
    std::vector<std::pair<Frame*,btRigidBody*> > _frameToBtBody;
};

class btVelocityDevice: public BtSimulator::btDevice {
public:

    btVelocityDevice(RigidDevice *rdev,std::vector<btTypedConstraint*> constraints):
        _rdev(rdev),
        _constraints(constraints)
    {

    }

    virtual ~btVelocityDevice(){};

    void update(double dt, rw::kinematics::State& state){
        rw::math::Q velQ = _rdev->getVelocity(state);
        std::cout << "size of velQ: " << velQ.size() << std::endl;
        for(int i = 0; i<_constraints.size(); i++){
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

    void postUpdate(rw::kinematics::State& state){
        rw::math::Q q = _rdev->getModel().getQ(state);
        for(int i = 0; i<_constraints.size(); i++){
            if(dynamic_cast<btHingeConstraint*>(_constraints[i])){
                btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(_constraints[i]);
                q[i] = hinge->getHingeAngle();
            } else if( dynamic_cast<btSliderConstraint*>(_constraints[i]) ){
                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(_constraints[i]);
               // q[i]
            }
        }
        _rdev->getModel().setQ(q, state);
    }

private:
    RigidDevice *_rdev;
    std::vector<btTypedConstraint*> _constraints;

};

BtSimulator::BtSimulator(dynamics::DynamicWorkCell *dwc):
	_dwc(dwc),_time(0.0),_dt(1.0/120.0)
{
}

#define MAX_VEL 2
#define MAX_FORCE 100

void BtSimulator::step(double dt, rw::kinematics::State& state){

	// update KINEMATICS OBJECT frames, links are moddeled this way
	// these need to be set before we make the update
    /*std::cout << "1";
	for(size_t i=0; i<_btLinks.size(); i++){
		Transform3D<> t3d = Kinematics::WorldTframe( &(_rwLinks[i]->getBodyFrame()), state);
		_btLinks[i]->getMotionState()->setWorldTransform( makeBtTransform(t3d) );
	}
	*/
    //std::cout << "Controller" << std::endl;
    //BOOST_FOREACH(Controller *controller, _dwc->getControllers() ){
    //    controller->update(dt, state);
    //}
    //std::cout << "Dev" << std::endl;
    BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->update(dt,state);
    }
    //std::cout << "Step" << std::endl;
	// update all device force/velocity input

	///step the simulation
	if (m_dynamicsWorld)
	{
		//double dt = 0.005;
		m_dynamicsWorld->stepSimulation(dt,200,1.0/480.0);
	    //m_dynamicsWorld->stepSimulation(_dt);
		_time += dt;
	}


    BOOST_FOREACH(btDevice *dev, _btDevices){
        dev->postUpdate(state);
    }

	// now copy all transforms into state
	for(size_t i=0; i<_btBodies.size(); i++){
		RigidBody *b = _rwBodies[i];
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

btRigidBody* BtSimulator::createRigidBody(Frame* bframe,
                                          double mass,
                                          const rw::kinematics::State& state,
                                          double margin){
    std::cout << "RigidBody: " << bframe->getName() << std::endl;
    // create a triangle mesh for all staticly connected nodes
    // TODO: check if colinfo for rigid body frame
    std::vector<Frame*> frames = DynamicUtil::getAnchoredFrames( *bframe, state);

    btCollisionShape* colShape =
        getColShapes(frames, bframe, _colCache, state, margin);
    if( colShape==NULL)
        return NULL;
    // if the colShape allready is in the m_collisionShapes then don't add it
    bool isInList = false;
    for(int m=0;m<m_collisionShapes.size();m++){
        if( m_collisionShapes[m]==colShape )
            isInList = true;
    }
    if( !isInList ){
        std::cout << "Col shape added!!!" << std::endl;
        m_collisionShapes.push_back(colShape);
    }

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    Transform3D<> t3d = Kinematics::worldTframe(bframe, state);
    btTransform startTransform = makeBtTransform(t3d);

    btVector3 localInertia(1,1,1);
    if (isDynamic)
        colShape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
    rbInfo.m_additionalDamping = true;
    rbInfo.m_additionalDampingFactor = 1.5;
    rbInfo.m_friction = 0.5;
    rbInfo.m_restitution = 0.01;

    btRigidBody* btbody = new btRigidBody(rbInfo);
    m_dynamicsWorld->addRigidBody(btbody);

    // add rigid body to to map

    _rwBtBodyToFrame[btbody] = bframe;
    if( isDynamic ){
        _rwFrameToBtBody[bframe] = btbody;
        BOOST_FOREACH(Frame* frame, frames){
            _rwFrameToBtBody[frame] = btbody;
        }
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

bool BtSimulator::setContactDetector(rw::common::Ptr<ContactDetector> detector) {
	return false;
}

void BtSimulator::initPhysics(rw::kinematics::State& state)
{
	//setCameraDistance(btScalar(50.));
    int nrFixedBody(0),nrRigidBody(0),nrLinkBody(0);
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_dispatcher->setNearCallback(MyNearCallback);

	btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

	///the maximum size of the collision world. Make sure objects stay within these boundaries
	///Don't make the world AABB size too large, it will harm simulation quality and performance
	btVector3 worldAabbMin(-20,-20,-20);
	btVector3 worldAabbMax(20,20,20);
	m_overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,MAX_PROXIES);

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol =
	    new btSequentialImpulseConstraintSolver();

	//btOdeQuickstepConstraintSolver* sol = new btOdeQuickstepConstraintSolver();
	m_solver = sol;

	btDiscreteDynamicsWorld *discreteWorld= new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = discreteWorld;
	discreteWorld->getSolverInfo().m_splitImpulse = true;
	//discreteWorld->getSolverInfo().m_numIterations = 20;
	//discreteWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.002;

	//discreteWorld->getSolverInfo().m_erp = 0.4f;
	//discreteWorld->getSolverInfo().m_erp2 = 0.3f;
	m_dynamicsWorld->setGravity(btVector3(0,0,-10));

	std::vector<RigidBody::Ptr> rbodies = _dwc->findBodies<RigidBody>();
	for(size_t i=0; i<rbodies.size(); i++){
		std::string fname = rbodies[i]->getBodyFrame().getName();
		std::cout << "Body: " << fname << std::endl;
		std::cout << "RigidBody: " << rbodies[i]->getBodyFrame().getName() << std::endl;
		// create a triangle mesh for all staticly connected nodes
		btRigidBody *btbody = createRigidBody(rbodies[i], state, 0.01);
		_btBodies.push_back(btbody);
		_rwBodies.push_back(rbody);
		nrRigidBody++;
	}

	std::vector<FixedBody::Ptr> fbodies = _dwc->findBodies<FixedBody>();
	for(size_t i=0; i<fbodies.size(); i++){
		std::string fname = fbodies[i]->getBodyFrame().getName();
		std::cout << "Body: " << fname << std::endl;

		std::cout << "FixedBody: " << fbodies[i]->getBodyFrame().getName() << std::endl;

		btRigidBody *btbody = createRigidBody(fbodies[i],0.f, state,0.01);
		btbody->setCollisionFlags( btbody->getCollisionFlags() |
								   btCollisionObject::CF_STATIC_OBJECT);

		nrFixedBody++;
	}

	std::vector<KinematicBody::Ptr> kbodies = _dwc->findBodies<KinematicBody>();
	for(size_t i=0; i<kbodies.size(); i++){
			// use kinematic objects for visualising
			std::cout << "KinematicBody: " << kbodies[i]->getBodyFrame().getName() << std::endl;

            btRigidBody *btbody = createRigidBody(kbodies[i], 0.f, state, 0.01);
            btbody->setCollisionFlags( btbody->getCollisionFlags() |
                                       btCollisionObject::CF_KINEMATIC_OBJECT );
            btbody->setActivationState(DISABLE_DEACTIVATION);

			nrLinkBody++;
		}
	}



	// ok, now add constraints
	BOOST_FOREACH(DynamicDevice::Ptr device, _dwc->getDynamicDevices() ){
	   if( device.cast<RigidDevice>()!=NULL ){
	        std::cout << "RigidDevice...." << std::endl;
	        // add kinematic constraints from base to joint1, joint1 to joint2 and so forth
	        RigidDevice::Ptr fDev = device.cast<RigidDevice>()
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
	            Frame *parent = joint->getParent(state);

	            std::cout << parent->getName() << "-->" << joint->getName() << std::endl;

	            btRigidBody *btParent = _rwFrameToBtBody[parent];
	            const Frame* parentFrame = NULL;
                if(btParent==NULL){
                    RW_WARN("btParent is NULL, " << joint->getName());
                } else {
                    parentFrame = _rwBtBodyToFrame[btParent];
                }

	            btRigidBody *btChild = createRigidBody(joint, 0.2 , state, 0.001 ); //_rwFrameToBtBody[joint];
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
	                double softness = 0.8;
	                double bias = 0.5;
	                double relaxation = 0.1;
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

	    } else  if( dynamic_cast<KinematicDevice*>( device ) ){
	        std::cout << "KinematicDevice...." << std::endl;
	        KinematicDevice* kdev = dynamic_cast<KinematicDevice*>( device );
	        std::vector<FrameBodyPair> frameBodyList;
	        BOOST_FOREACH(KinematicBody *body, kdev->getLinks() ){
	            Frame *frame = &body->getBodyFrame();
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

    std::cout << "BtSimulator: initializing physics" << std::endl
              << " - Collision Margin: " << _dwc->getCollisionMargin() << std::endl
              << " - Nr of fixed bodies: " << nrFixedBody << std::endl
              << " - Nr of rigid bodies: " << nrRigidBody << std::endl
              << " - Nr of controlled bodies: " << nrLinkBody << std::endl;

}

void BtSimulator::resetScene(rw::kinematics::State& state)
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
        RigidBody *b = _rwBodies[i];
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

// if both GripA and GribB exist, create a prismatic joint between them
/*if( gripA!=NULL && gripB!=NULL && gripperBase!=NULL){
    gripA->setActivationState(DISABLE_DEACTIVATION);
    gripB->setActivationState(DISABLE_DEACTIVATION);
    btTransform sliderTransform;

    // bestem transformationen fra gripperBase til
    Transform3D<> baseTframeA = Kinematics::frameTframe(gripperBaseFrame,gripAFrame,state);

    btTransform frameInA, frameInB;
    frameInA = btTransform::getIdentity();
    frameInB = makeBtTransform(baseTframeA);


    bool useLinearReferenceFrameA = true;//use gripB for linear limits
    btGeneric6DofConstraint* slider = new btGeneric6DofConstraint(*gripA,*gripperBase,frameInA,frameInB,useLinearReferenceFrameA);
    slider->setLinearLowerLimit(btVector3(0,0.0,0));
    slider->setLinearUpperLimit(btVector3(0,0.1,0));
    slider->setAngularLowerLimit(btVector3(0,0,0));
    slider->setAngularUpperLimit(btVector3(0,0,0));

    m_dynamicsWorld->addConstraint(slider);

    Transform3D<> baseTframeB = Kinematics::frameTframe(gripperBaseFrame,gripBFrame,state);
    frameInA = btTransform::getIdentity();
    frameInB = makeBtTransform(baseTframeB);

    //bool useLinearReferenceFrameA = false;//use gripB for linear limits
    slider = new btGeneric6DofConstraint(*gripB,*gripperBase,frameInA,frameInB,useLinearReferenceFrameA);
    slider->setLinearLowerLimit(btVector3(0,-0.1,0));
    slider->setLinearUpperLimit(btVector3(0,0.00,0));
    slider->setAngularLowerLimit(btVector3(0,0,0));
    slider->setAngularUpperLimit(btVector3(0,0,0));

    m_dynamicsWorld->addConstraint(slider);
    // add yet another constraint such that
    _usingGripper = true;
}
*/


/*
 *
 *
void BtSimulator::adjustForceOnGripper( rw::kinematics::State& state ){
    Transform3D<> baseTframeA = Kinematics::frameTframe(gripperBaseFrame,gripAFrame,state);
    Transform3D<> baseTframeB = Kinematics::frameTframe(gripperBaseFrame,gripBFrame,state);

    double posA = baseTframeA.P()[1];
    double posB = baseTframeB.P()[1];




    btVector3 velA = gripA->getVelocityInLocalPoint(btVector3(0,0,0));
    btVector3 velB = gripB->getVelocityInLocalPoint(btVector3(0,0,0));

    //std::cout << "VelA: " << velA[0] << ";" << velA[1] <<  ";" << velA[2] <<   std::endl;

    // position is the same since its a parallel gripper with coupled joints
    double gPosA = Math::clamp(_goalPos, 0, 0.1);
    double gPosB = -Math::clamp(_goalPos, 0, 0.1);
    // compensate for position difference
    double diffPosComp = (posA + posB);

    std::cout << "PosA: " << posA << " -> " << gPosA << std::endl;
    std::cout << "PosB: " << posB << " -> " << gPosB << std::endl;

    double errPA = posA-gPosA;
    double errPB = posB-gPosB;

    std::cout << "errPosA: " << errPA << std::endl;
    std::cout << "errPosB: " << errPB << std::endl;

    double errPosA = errPA*_qKp + (errPA - _errPALast)*_qKd/_dt ;
    _errPALast = errPA;

    double errPosB = errPB*_qKp + (errPB - _errPBLast)*_qKd/_dt ;
    _errPBLast = errPB;

    errPosA = Math::clamp(errPosA, -MAX_VEL, MAX_VEL);
    errPosB = Math::clamp(errPosB, -MAX_VEL, MAX_VEL);


    double forceA = (velA[1]-errPosA)*_vKp + (velA[1]-errPosA)*_vKd/_dt;
    double forceB = (velB[1]-errPosB)*_vKp + (velB[1]-errPosB)*_vKd/_dt;

    forceA = Math::clamp( forceA, -MAX_FORCE, MAX_FORCE );
    forceB = Math::clamp( forceB, -MAX_FORCE, MAX_FORCE );

    std::cout << "Force: " << forceA << " " << forceB << std::endl;

    // transform local description of force to global description
    Transform3D<> wTa = Kinematics::worldTframe(gripAFrame,state);
    Transform3D<> wTb = Kinematics::worldTframe(gripBFrame,state);
    Vector3D<> vfA = wTa * Vector3D<>(0, forceA,0);
    Vector3D<> vfB = wTb * Vector3D<>(0, forceB,0);
    gripA->clearForces();
    gripA->applyCentralForce(makeBtVector(vfA));
    gripB->clearForces();
    gripB->applyCentralForce(makeBtVector(vfB));
}*/

/*
Transform3D<> wTchild = Kinematics::worldTframe(joint,state);
Transform3D<> wTparent = Kinematics::worldTframe(parentFrame,state);

btTransform frameInA, frameInB;
frameInA.setIdentity(); // the joint allready use z-axis for rotation
frameInB = makeBtTransform( inverse(wTparent) * wTchild ); // calculate transform from parent to joint

btChild->setActivationState(DISABLE_DEACTIVATION);
btParent->setActivationState(DISABLE_DEACTIVATION);

btHingeConstraint *hinge = new btHingeConstraint(*btChild, *btParent, frameInA, frameInB);
hinge->setLimit( joint->getBounds().first, joint->getBounds().second );
//hinge->setAngularOnly(true);
hinge->enableAngularMotor(true, 0.0, 10);
m_dynamicsWorld->addConstraint(hinge, true);
_jointToConstraintMap[joint] = hinge;
constraints.push_back(hinge);*/
