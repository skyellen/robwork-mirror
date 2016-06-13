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

#include "BtBody.hpp"
#include "BtUtil.hpp"
#include "BtMaterial.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Plane.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <LinearMath/btDefaultMotionState.h>

using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsimlibs::bullet;

#define COL_MARGIN 0.001

BtBody::BtBody(Body::Ptr body,
		const MaterialDataMap* frictionMap, const ContactDataMap* collisionMap,
		btDynamicsWorld* btWorld, const State& state):
	_rwBody(body),
	_rwBodyDynamic(_rwBody.cast<RigidBody>()),
	_rwBodyKinematic(_rwBody.cast<KinematicBody>()),
	_rwBodyStatic(_rwBody.cast<FixedBody>()),
	_btDynamicsWorld(btWorld),
	_bTcom(isDynamic() ? Transform3D<>(body->getInfo().masscenter,_rwBodyDynamic->getBodyPrincipalInertia().first) : Transform3D<>::identity()),
	_collisionShape(getColShape(body, _bTcom)),
	_btRigidBody(createRigidBody(frictionMap, collisionMap, body,state))
{
}

BtBody::~BtBody() {
	if (_collisionShape != NULL) {
		for (int i = 0; i < _collisionShape->getNumChildShapes(); i++) {
			btCollisionShape* const shape = _collisionShape->getChildShape(i);
			if (btGImpactMeshShape* const gimpact = dynamic_cast<btGImpactMeshShape*>(shape)) {
				delete gimpact->getMeshInterface();
			}
			RW_ASSERT(shape->getUserPointer());
			GeometryMetaData* const data = static_cast<GeometryMetaData*>(shape->getUserPointer());
			RW_ASSERT(data);
			delete data;
			delete shape;
		}
		delete _collisionShape;
	}
	if (_btRigidBody != NULL) {
		RW_ASSERT(_btRigidBody->getUserPointer());
		BodyMetaData* const data = static_cast<BodyMetaData*>(_btRigidBody->getUserPointer());
		RW_ASSERT(data);
		delete data;
		_btDynamicsWorld->removeRigidBody(_btRigidBody);
		if (_btRigidBody->getMotionState()){
			delete _btRigidBody->getMotionState();
		}
		delete _btRigidBody;
	}
}

Body::Ptr BtBody::getRwBody() const {
	return _rwBody;
}

btRigidBody* BtBody::getBulletBody() const {
	return _btRigidBody;
}

void BtBody::update(double dt, State& state) const {
	if (_rwBodyKinematic) {
		Transform3D<> t3d = Kinematics::worldTframe(_rwBodyKinematic->getBodyFrame(), state);
		t3d.P() += _rwBodyKinematic->getLinVelW(state)*dt;
		t3d.R() = EAA<>( _rwBodyKinematic->getAngVelW(state)*dt).toRotation3D() * t3d.R();
		_btRigidBody->getMotionState()->setWorldTransform( BtUtil::makeBtTransform(t3d) );
	}
}

void BtBody::postupdate(State& state) const {
	if (isDynamic()) {
		MovableFrame &mframe = *(_rwBodyDynamic->getMovableFrame());
		const Transform3D<> wTcom = getWorldTcom();
		const Transform3D<> wTb = wTcom*inverse(_bTcom);

		const Vector3D<> ang = BtUtil::toVector3D( _btRigidBody->getAngularVelocity() );
		const Vector3D<> lin = BtUtil::toVector3D( _btRigidBody->getLinearVelocity() ) + cross(ang,wTb.P()-wTcom.P());

		Transform3D<> wTp = Kinematics::worldTframe(mframe.getParent(state), state);
		mframe.setTransform( inverse(wTp) * wTcom*inverse(_bTcom), state );

		_rwBodyDynamic->setAngVelW( ang , state);
		_rwBodyDynamic->setLinVelW( lin , state);
    } else if (_rwBodyKinematic) {
		MovableFrame &mframe = *(_rwBodyKinematic->getMovableFrame());
		const Transform3D<> wTcom = getWorldTcom();
		const Transform3D<> wTb = wTcom*inverse(_bTcom);

		const Vector3D<> ang = BtUtil::toVector3D( _btRigidBody->getAngularVelocity() );
		const Vector3D<> lin = BtUtil::toVector3D( _btRigidBody->getLinearVelocity() ) + cross(ang,wTb.P()-wTcom.P());

		const Transform3D<> wTp = Kinematics::worldTframe(mframe.getParent(state), state);
		mframe.setTransform( inverse(wTp) * wTcom, state );

		_rwBodyKinematic->setAngVelW( ang , state);
		_rwBodyKinematic->setLinVelW( lin , state);
    }
}

bool BtBody::isDynamic() const {
	return _rwBodyDynamic != NULL;
}

const Transform3D<>& BtBody::getBodyTcom() const {
	return _bTcom;
}

Transform3D<> BtBody::getWorldTcom() const {
	const btVector3 &v = _btRigidBody->getCenterOfMassTransform().getOrigin();
	const btQuaternion &q = _btRigidBody->getCenterOfMassTransform().getRotation();
	return BtUtil::toTransform3D(v,q);
}

btRigidBody* BtBody::createRigidBody(
		const MaterialDataMap* frictionMap, const ContactDataMap* collisionMap,
		Body::Ptr body, const State& state) const
{
	const Transform3D<> wTb = Kinematics::worldTframe(_rwBody->getBodyFrame(),state);
	const Transform3D<> t3d = wTb*_bTcom;
	const btTransform startTransform = BtUtil::makeBtTransform(t3d);
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//note however that currently the postupdate does not make use of the motion state interpolation
	// (it uses information directly from the body instead, which corresponds to the last simulation step)
	btDefaultMotionState* const myMotionState = new btDefaultMotionState(startTransform);
	double mass = 0;
	btVector3 principalInertia(0,0,0);
	if (isDynamic()) {
		mass = _rwBodyDynamic->getMass();
		principalInertia = BtUtil::makeBtVector(_rwBodyDynamic->getBodyPrincipalInertia().second);
	}
	btRigidBody::btRigidBodyConstructionInfo rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,_collisionShape,principalInertia);
	// TODO: should damping be enabled at what should the value be?
	//    rbInfo.m_additionalDamping = true;
	//    rbInfo.m_additionalDampingFactor = 1.5;

	// TODO: simat - fix friction
	// bullets set "friction" on objects and as default when bodies collidied the 2 m_friction values are multiplied together. This is not correct and is currently overridden in the CustomMaterialCombinerCallback()
	//rbInfo.m_friction = 0.5;
	//rbInfo.m_restitution = 0.01;

	btRigidBody* const btbody = new btRigidBody(rbInfo);
    if (!_rwBodyStatic) {
    	const Vector3D<> angVel = body->getAngVelW(state);
    	const Vector3D<> linVel = body->getLinVelW(state)+cross(angVel,t3d.P()-wTb.P());
    	btbody->setLinearVelocity(BtUtil::makeBtVector(linVel));
    	btbody->setAngularVelocity(BtUtil::makeBtVector(angVel));
    }

    BtMaterial* const material = new BtMaterial(frictionMap,body->getMaterialID(),collisionMap,body->getInfo().objectType);
    BodyMetaData* const data = new BodyMetaData(_rwBody->getBodyFrame(),material); // takes ownership of material
    btbody->setUserPointer(data);

	_btDynamicsWorld->addRigidBody(btbody);

	if (_rwBodyStatic) {
		btbody->setCollisionFlags( btbody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT );
	} else if (_rwBodyKinematic) {
        btbody->setCollisionFlags( btbody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
        btbody->setActivationState(DISABLE_DEACTIVATION);
	} else if (_rwBodyDynamic) {
    	btbody->setFlags(btbody->getFlags() | BT_ENABLE_GYROPSCOPIC_FORCE);
        btbody->setActivationState(DISABLE_DEACTIVATION);
	}
	btbody->setCollisionFlags(btbody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);// flag set to enable the CustomMaterialCombinerCallback to handle friction- and restitution coefficients

	return btbody;
}

btCollisionShape* BtBody::createColShape(rw::common::Ptr<const Geometry> geometry) const {
	btCollisionShape* shape = NULL;
	if(Cylinder* const cyl = dynamic_cast<Cylinder*>(geometry->getGeometryData().get()) ){
		const btVector3 halfExtents(cyl->getRadius(),0,cyl->getHeight()/2.);
		shape = new btCylinderShapeZ(halfExtents);
	} else if(Sphere* const sphere = dynamic_cast<Sphere*>(geometry->getGeometryData().get()) ){
		shape = new btSphereShape(sphere->getRadius());
	} else if(Plane* const plane = dynamic_cast<Plane*>(geometry->getGeometryData().get()) ){
		const btVector3 n = BtUtil::makeBtVector(plane->normal());
		shape = new btStaticPlaneShape(n,plane->d());
	} else {
		const TriMesh::Ptr mesh = geometry->getGeometryData()->getTriMesh();
		btTriangleMesh* const trimesh = new btTriangleMesh();

		for (size_t i=0; i<mesh->getSize(); i++)
		{
			const Triangle<> tri = mesh->getTriangle(i);
			btVector3 v1(tri[0][0], tri[0][1], tri[0][2]);
			btVector3 v2(tri[1][0], tri[1][1], tri[1][2]);
			btVector3 v3(tri[2][0], tri[2][1], tri[2][2]);
			trimesh->addTriangle(v1, v2, v3);
		}
		if (trimesh->getNumTriangles() == 0) {
			delete trimesh;
			return NULL;
		}
		// create the collision shape from the trimesh data
		btGImpactMeshShape* const colShape = new btGImpactMeshShape(trimesh);
		colShape->setMargin(COL_MARGIN);
		colShape->postUpdate();
		colShape->updateBound();// Call this method once before doing collisions
		shape = colShape;
	}

	if (shape)
		shape->setUserPointer(new GeometryMetaData(geometry));

	return shape;
}

btCompoundShape* BtBody::getColShape(Body::Ptr body, const Transform3D<>& bTcom) const {
	btCompoundShape* const composite = new btCompoundShape();
	BOOST_FOREACH(const Geometry::Ptr geometry, body->getGeometry()) {
		const Transform3D<> rw_comTgeo = inverse(bTcom)*geometry->getTransform();
		const btTransform comTgeo = BtUtil::makeBtTransform( rw_comTgeo );
		btCollisionShape* const colShape = createColShape(geometry);
		if (colShape != NULL) {
			composite->addChildShape(comTgeo, colShape);
		}
	}
	if (composite->getNumChildShapes() == 0) {
		delete composite;
		return NULL;
	}
    return composite;
}

BtBody::BodyMetaData::BodyMetaData(const Frame* frame, const BtMaterial* material):
	frame(frame),
	material(material)
{
}

BtBody::BodyMetaData::~BodyMetaData() {
	if (material)
		delete material;
}

BtBody::GeometryMetaData::GeometryMetaData(rw::common::Ptr<const Geometry> geometry):
	geometry(geometry)
{
}

BtBody::GeometryMetaData::~GeometryMetaData() {
}
