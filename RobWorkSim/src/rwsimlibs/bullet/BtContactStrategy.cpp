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

#include "BtContactStrategy.hpp"
#include "BtUtil.hpp"

#include <rw/geometry/Geometry.hpp>

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Plane.hpp>

#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>

#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>

#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>

#include <BulletCollision/Gimpact/btGImpactShape.h>

#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <LinearMath/btDefaultMotionState.h>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;
using namespace rwsimlibs::bullet;

namespace {
class BtContactModel: public ContactModel {
public:
	typedef rw::common::Ptr<BtContactModel> Ptr;
	BtContactModel(BtContactStrategy* pOwner): ContactModel(pOwner) { setFrame(NULL); };
	virtual ~BtContactModel() {
		clear();
	};
	std::string getName() const {
		return "BtContactModel";
	}
	void clear() {
		models.clear();
	}
	bool remove(const std::string& geomId) {
		bool removed = false;
		for (std::vector<BtContactModel::BtModel>::iterator it = models.begin(); it != models.end(); it++) {
			if (it->geoId == geomId) {
				models.erase(it);
				removed = true;
				break;
			}
		}
		return removed;
	}

public:
	struct BtModel {
		BtModel(): frame(NULL), body(NULL), bodyAlternative(NULL), movable(true) {}
		~BtModel() {
			const bool bodyAltEq = body == bodyAlternative;
			if (body) {
				const btMotionState* const myMotionState = body->getMotionState();
				const btCollisionShape* const shape = body->getCollisionShape();
				delete body;
				delete shape;
				delete myMotionState;
				body = 0;
			}
			if (!bodyAltEq && bodyAlternative) {
				const btMotionState* const myMotionState = bodyAlternative->getMotionState();
				const btCollisionShape* const shape = bodyAlternative->getCollisionShape();
				delete body;
				delete shape;
				delete myMotionState;
				bodyAlternative = 0;
			}
		}
		std::string geoId;
		Transform3D<> transform;
		const Frame* frame;
		btRigidBody* body;
		btRigidBody* bodyAlternative; // alternative body to use for certain special primitive pairs
		bool movable;
	};

	BtModel& newModel() {
		models.resize(models.size()+1);
		return models.back();
	}

	void abortNewModel() {
		models.resize(models.size()-1);
	}

	const std::vector<BtModel>& getModels() const {
		return models;
	}

private:
	std::vector<BtModel> models;
};
}

BtContactStrategy::BtContactStrategy():
	_dispatcher(new btCollisionDispatcher(new btDefaultCollisionConfiguration()))
{
}

BtContactStrategy::~BtContactStrategy() {
	const btCollisionConfiguration*	config = _dispatcher->getCollisionConfiguration();
	delete _dispatcher;
	delete config;
}

bool BtContactStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	// List of primitive pairs seperately considered
	// Ball-Ball
	if (geoA->getType() == GeometryData::SpherePrim && geoB->getType() == GeometryData::SpherePrim)
		return true;
	// Ball-Plane
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::SpherePrim)
		return true;
	if (geoA->getType() == GeometryData::SpherePrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	// Cylinder-Plane
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::CylinderPrim)
		return true;
	if (geoA->getType() == GeometryData::CylinderPrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	// Plane-Box
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::BoxPrim)
		return true;
	if (geoA->getType() == GeometryData::BoxPrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	// Tube-Plane
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::TubePrim)
		return true;
	if (geoA->getType() == GeometryData::TubePrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	// Cylinder-Tube
	if (geoA->getType() == GeometryData::CylinderPrim && geoB->getType() == GeometryData::TubePrim)
		return true;
	if (geoA->getType() == GeometryData::TubePrim && geoB->getType() == GeometryData::CylinderPrim)
		return true;
	// For now just match remaining
	return true;
}

std::vector<Contact> BtContactStrategy::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data, ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	const BtContactModel::Ptr mA = a.cast<BtContactModel>();
	const BtContactModel::Ptr mB = b.cast<BtContactModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);

	std::vector<Contact> res;
	BOOST_FOREACH(const BtContactModel::BtModel& modelA, mA->getModels()) {
		BOOST_FOREACH(const BtContactModel::BtModel& modelB, mB->getModels()) {
			const btTransform transformA = BtUtil::makeBtTransform(wTa*modelA.transform);
			const btTransform transformB = BtUtil::makeBtTransform(wTb*modelB.transform);
			btRigidBody* bodyA = modelA.body;
			btRigidBody* bodyB = modelB.body;
			if (modelA.bodyAlternative && modelB.bodyAlternative) {
				bodyA = modelA.bodyAlternative;
				bodyB = modelB.bodyAlternative;
			}
#if BT_BULLET_VERSION > 281
			const btCollisionObjectWrapper obj0Wrap(0,bodyA->getCollisionShape(),bodyA,transformA,-1,-1);
			const btCollisionObjectWrapper obj1Wrap(0,bodyB->getCollisionShape(),bodyB,transformB,-1,-1);
#else
			const btCollisionObjectWrapper obj0Wrap(0,bodyA->getCollisionShape(),bodyA,transformA);
			const btCollisionObjectWrapper obj1Wrap(0,bodyB->getCollisionShape(),bodyB,transformB);
#endif
			btCollisionAlgorithm* algorithm = _dispatcher->findAlgorithm(&obj0Wrap,&obj1Wrap);
			btManifoldArray manifolds;
			algorithm->getAllContactManifolds(manifolds);
			for (int i = 0; i < manifolds.size(); i++) {
				manifolds[i]->setContactBreakingThreshold(getThreshold());
			}

			btManifoldResult contactPointResult(&obj0Wrap,&obj1Wrap);

			btDispatcherInfo dispatchInfo;
			algorithm->processCollision(&obj0Wrap,&obj1Wrap,dispatchInfo,&contactPointResult);

			const btPersistentManifold* const manifold = contactPointResult.getPersistentManifold();
			const bool swapped = (manifold->getBody0() == bodyA) ? false : true;
			for (int i = 0; i < manifold->getNumContacts(); i++) {
				const btManifoldPoint& c = manifold->getContactPoint(i);

				const btVector3 pA = c.getPositionWorldOnA();
				const btVector3 pB = c.getPositionWorldOnB();
				const btVector3 n = c.m_normalWorldOnB;

				Contact contact;
				contact.setDepth(-c.m_distance1);
				if (swapped) {
					contact.setNormal(BtUtil::toVector3D(n));
					contact.setPoints(BtUtil::toVector3D(pB),BtUtil::toVector3D(pA));
				} else {
					contact.setNormal(-BtUtil::toVector3D(n));
					contact.setPoints(BtUtil::toVector3D(pA),BtUtil::toVector3D(pB));
				}
				contact.setFrameA(modelA.frame);
				contact.setFrameB(modelB.frame);
				contact.setModelA(mA);
				contact.setModelB(mB);
				contact.setTransform(inverse(wTa)*wTb);
				res.push_back(contact);
			}
		}
	}
	return res;
}

std::vector<Contact> BtContactStrategy::updateContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data, ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> res;
	RW_THROW("BtContactStrategy does not implement udpateContacts!");
	return res;
}

std::string BtContactStrategy::getName() {
	return "BtContactStrategy";
}

ProximityModel::Ptr BtContactStrategy::createModel() {
	return ownedPtr(new BtContactModel(this));
}

void BtContactStrategy::destroyModel(ProximityModel* model) {
	BtContactModel* bmodel = dynamic_cast<BtContactModel*>(model);
	RW_ASSERT(bmodel);
	bmodel->clear();
}

bool BtContactStrategy::addGeometry(ProximityModel* model, const Geometry& geom) {
	BtContactModel* bmodel = dynamic_cast<BtContactModel*>(model);
	RW_ASSERT(bmodel);
	GeometryData::Ptr geomData = geom.getGeometryData();

	BtContactModel::BtModel& newModel = bmodel->newModel();
	newModel.geoId = geom.getId();
	newModel.transform = geom.getTransform();
	newModel.frame = geom.getFrame();

	btCollisionShape* shape = NULL;
	if(Cylinder* const cyl = dynamic_cast<Cylinder*>(geomData.get()) ){
		const btVector3 halfExtents(cyl->getRadius(),0,cyl->getHeight()/2.);
		shape = new btCylinderShapeZ(halfExtents);
		newModel.movable = true;
	} else if(Sphere* const sphere = dynamic_cast<Sphere*>(geomData.get()) ){
		shape = new btSphereShape(sphere->getRadius());
		newModel.movable = true;
	} else if(Box* const box = dynamic_cast<Box*>(geomData.get()) ){
		const btVector3 halfExtents(box->getParameters()[0]/2.,box->getParameters()[1]/2.,box->getParameters()[2]/2.);
		shape = new btBoxShape(halfExtents);
		newModel.movable = true;
	} else if(Plane* const plane = dynamic_cast<Plane*>(geomData.get()) ){
		const btVector3 n = BtUtil::makeBtVector(plane->normal());
		shape = new btStaticPlaneShape(n,plane->d());
		newModel.movable = false;
	} else {
		const TriMesh::Ptr mesh = geomData->getTriMesh();
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
			bmodel->abortNewModel();
			delete trimesh;
			return NULL;
		}
		// create the collision shape from the trimesh data
		btGImpactMeshShape* const colShape = new btGImpactMeshShape(trimesh);
		colShape->postUpdate();
		colShape->updateBound();// Call this method once before doing collisions

		shape = colShape;
		newModel.movable = true;
	}
	if (shape) {
		//shape->setMargin(getThreshold());
		shape->setMargin(0);

		// Create dummy body
		btDefaultMotionState* const myMotionState = new btDefaultMotionState();
		const double mass = 1.; // must be greater than zero to be non-static (to avoid warning)
		const btVector3 principalInertia(0,0,0);
		const btRigidBody::btRigidBodyConstructionInfo rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,shape,principalInertia);
		btRigidBody* const btbody = new btRigidBody(rbInfo);
        btbody->setActivationState(DISABLE_DEACTIVATION);

        newModel.body = btbody;

		/*// Create alternative shapes
		btCollisionShape* shapeAlt = NULL;
		if(Tube* const tube = dynamic_cast<Tube*>(geomData.get()) ){
			const btVector3 halfExtents(tube->getInnerRadius()+tube->getThickness(),0,tube->getHeight()/2.);
			shapeAlt = new btCylinderShapeZ(halfExtents);
		} else if(Plane* const plane = dynamic_cast<Plane*>(geomData.get()) ){
			bmodel->models.back().bodyAlternative = bmodel->models.back().body;
		}
		if (shapeAlt) {
			shapeAlt->setMargin(0);

			// Create dummy body
			btDefaultMotionState* const myMotionState = new btDefaultMotionState();
			const btRigidBody::btRigidBodyConstructionInfo rbInfo = btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,shapeAlt,principalInertia);
			btRigidBody* const btbody = new btRigidBody(rbInfo);
	        btbody->setActivationState(DISABLE_DEACTIVATION);

			bmodel->models.back().bodyAlternative = btbody;
		}*/

		return true;
	} else {
		bmodel->abortNewModel();
		return false;
	}
}

bool BtContactStrategy::addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy) {
	// forceCopy ignored: data is always copied to corresponding bullet collision shapes.
	return addGeometry(model, *geom);
}

bool BtContactStrategy::removeGeometry(ProximityModel* model, const std::string& geomId) {
	BtContactModel* bmodel = dynamic_cast<BtContactModel*>(model);
	return bmodel->remove(geomId);
}

std::vector<std::string> BtContactStrategy::getGeometryIDs(ProximityModel* model) {
	BtContactModel* bmodel = dynamic_cast<BtContactModel*>(model);
	std::vector<std::string> res;
	const std::vector<BtContactModel::BtModel>& models = bmodel->getModels();
	for (std::vector<BtContactModel::BtModel>::const_iterator it = models.begin(); it != models.end(); it++) {
		res.push_back(it->geoId);
	}
	return res;
}

void BtContactStrategy::clear() {
	// Nothing to clear
}

double BtContactStrategy::getThreshold() const {
	const PropertyMap& pmap = _propertyMap;
	double threshold;
	try {
		threshold = pmap.get<double>("BtContactStrategyThreshold");
	} catch(const Exception&) {
		threshold = pmap.get<double>("MaxSepDistance", 0.0005);
	}
    return threshold;
}
