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


#include "ProximityStrategyBullet.hpp"

#include <rw/geometry/TriMesh.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>

#include <boost/foreach.hpp>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>

#include <rw/geometry/TriangleUtil.hpp>

#include <float.h>
#include <vector>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::loaders;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace
{
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

#ifdef skdjsakfjd
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
#endif

}

//----------------------------------------------------------------------
// ProximityStrategyBullet

ProximityStrategyBullet::ProximityStrategyBullet() :
    //_firstContact(true),
	_threshold(DBL_MAX)
{
	clearStats();
	// create dispatcher

	//_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
	_dispatcher = new	btCollisionDispatcher(_collisionConfiguration);

	//_gimpactColAlg = new btGImpactCollisionAlgorithm();
	btGImpactCollisionAlgorithm::registerAlgorithm(_dispatcher);


	//_dispatcher->setNearCallback(MyNearCallback);
	btVector3 worldAabbMin(-20,-20,-20);
	btVector3 worldAabbMax(20,20,20);
	btAxisSweep3 *overlappingPairCache = new btAxisSweep3(worldAabbMin,worldAabbMax,200);

	_cworld = new btCollisionWorld(_dispatcher, overlappingPairCache, _collisionConfiguration);


/*
	//dispatcher will keep algorithms persistent in the collision pair
	if (!collisionPair.m_algorithm)
	{
		collisionPair.m_algorithm = dispatcher.findAlgorithm(colObj0,colObj1);
	}

	if (collisionPair.m_algorithm)
	{
		btManifoldResult contactPointResult(colObj0,colObj1);

		if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
		{
			//discrete collision detection query
			collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
		} else
		{
			//continuous collision detection query, time of impact (toi)
			btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
			if (dispatchInfo.m_timeOfImpact > toi)
				dispatchInfo.m_timeOfImpact = toi;

		}
	}
*/
}


rw::proximity::ProximityModel::Ptr ProximityStrategyBullet::createModel()
{

	ProximityModelBullet *model = new ProximityModelBullet(this);
    return ownedPtr(model);
}

void ProximityStrategyBullet::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// TODO: though the models should probably be removed from cache
}



bool ProximityStrategyBullet::addGeometry(rw::proximity::ProximityModel* model,
                                       	   	    const rw::geometry::Geometry& geom) {


	std::cout << "Adding geometry" << std::endl;


    //std::cout << "add geometry: " << geom.getId() << std::endl;
    ProximityModelBullet *pmodel = (ProximityModelBullet*) model;

	GeometryData::Ptr gdata = geom.getGeometryData();
    // check if geomid is in model. remove it if it has
    BOOST_FOREACH(BulletModel &m, pmodel->models){
        if( m.geoid==geom.getId() ){
            removeGeometry( model, geom.getId() );
            break;
        }
    }

    rw::common::Ptr<btCollisionShape> colShape;

    // check if model is in
    CacheKey key(geom.getGeometryData().get(),geom.getScale());
    if( _modelCache.has(key) ){
        // use the cached model
    	std::cout << "using cache" << std::endl;
    	colShape = _modelCache.get(key);
    } else
    {


    	// create model from triangle mesh

		TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0){
        	std::cout << "bad mesh size!... " << std::endl;
            return false;
        }

    	// check if geometry is convex
        if( gdata->isConvex() ){
        	std::cout << "CONVEX mesh" << std::endl;
        	IndexedTriMeshN0<float>::Ptr imesh = rw::geometry::TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >( *mesh );

			rw::common::Ptr<btConvexHullShape> tmpshape = ownedPtr( new btConvexHullShape() );

        	std::vector<Vector3D<float> > &vertices = imesh->getVertices();
			for (size_t i=0; i<vertices.size(); i++)
			{
				btVector3 v = makeBtVector( cast<double>(vertices[i]) );
				tmpshape->addPoint( v );
			}

			tmpshape->setMargin(0.0005);
			colShape = tmpshape;
			_modelCache.add(key, colShape);

        } else {
        	std::cout << "NOT CONVEX mesh" << std::endl;
			const double scale = geom.getScale();
			//std::cout << "new trimesh" << std::endl;
			btTriangleMesh* trimesh = new btTriangleMesh();
			// TODO: remember to transform any geometry reference to root nodes reference
			for (size_t i=0; i<mesh->getSize(); i++)
			{
				Triangle<> tri = mesh->getTriangle(i);
				btVector3 v1(tri[0][0]*scale, tri[0][1]*scale, tri[0][2]*scale);
				btVector3 v2(tri[1][0]*scale, tri[1][1]*scale, tri[1][2]*scale);
				btVector3 v3(tri[2][0]*scale, tri[2][1]*scale, tri[2][2]*scale);
				trimesh->addTriangle(v1, v2, v3);
			}
			std::cout << "NUMBER TRIANGLES: " << trimesh->getNumTriangles() << std::endl;
			if (trimesh->getNumTriangles() == 0) {
				delete trimesh;
				return NULL;
			}

			// create the collision shape from the trimesh data
			//bool useQuantizedBvhTree = true;

			//rw::common::Ptr<btCollisionShape> tmpShape  = ownedPtr( new btBvhTriangleMeshShape(trimesh,useQuantizedBvhTree) );
			//tmpShape->setMargin(0.0005);


/*
			btGImpactConvexDecompositionShape *colShape  = new
			    btGImpactConvexDecompositionShape(
			           trimesh, btVector3(1.f,1.f,1.f), btScalar(margin) );
*/
			rw::common::Ptr<btGImpactMeshShape> tmpShape = ownedPtr( new btGImpactMeshShape(trimesh) );
			tmpShape->setMargin(0.0005);
			tmpShape->postUpdate();
			tmpShape->updateBound();// Call this method once before doing collisions


			colShape = tmpShape;
			_modelCache.add(key, colShape);
        }
    }

    rw::common::Ptr<btCollisionObject> colobj = ownedPtr( new btCollisionObject() );
    colobj->setCollisionShape( colShape.get() );
    colobj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
    colobj->setActivationState( 1 );
    _cworld->addCollisionObject( colobj.get() );

    BulletModel bmodel(geom.getId(), geom.getTransform(), colobj);
    bmodel.ckey = key;
    pmodel->models.push_back( bmodel );

    _allmodels.push_back(pmodel->models.back());
    _geoIdToModelIdx[geom.getId()].push_back(_allmodels.size()-1);
    return true;
}

bool ProximityStrategyBullet::addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy){
    // we allways copy the data here
    return addGeometry(model,*geom);
}

bool ProximityStrategyBullet::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
    //std::cout << "Remove geometry: " << geomId << std::endl;
	/*
    PQPProximityModel *pmodel = (PQPProximityModel*) model;
    // remove from model
    int idx=-1;
    for(size_t i=0;i<pmodel->models.size();i++)
        if(pmodel->models[i].geoid==geomId){
            idx = i;
            break;
        }
    if(idx<0){
        //RW_THROW("No geometry with id: \""<< geomId << "\" exist in proximity model!");
        return false;
    }
    // remove from cache
    _modelCache.remove(pmodel->models[idx].ckey);
    RWPQPModelList::iterator iter = pmodel->models.begin();
    for(;iter!=pmodel->models.end();++iter){
        if((*iter).geoid==geomId){
            pmodel->models.erase(iter);
            return true;
        }
    }
	return false;
	*/
}


ProximityStrategyBullet::QueryData ProximityStrategyBullet::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
	QueryData qdata;
    if(data.getCache()==NULL || data.getCache()->_owner!=this)
        data.getCache() = ownedPtr( new ProximityCacheBullet(this));

    qdata.cache = static_cast<ProximityCacheBullet*>(data.getCache().get());

    qdata.a = (ProximityModelBullet*)aModel.get();
    qdata.b = (ProximityModelBullet*)bModel.get();
    return qdata;
}

struct ContactCB: public btCollisionWorld::ContactResultCallback {

	btScalar addSingleResult(btManifoldPoint& cp,
							   const btCollisionObjectWrapper* colObj0Wrap,
							   int partId0,int index0,
							   const btCollisionObjectWrapper* colObj1Wrap,
							   int partId1,int index1)
	{
		//std::cout << "add single " << std::endl;
		contacts.push_back( cp.getPositionWorldOnA() );
		return 1.0;
	};
	std::vector<btVector3> contacts;
};

bool ProximityStrategyBullet::inCollision(
        ProximityModel::Ptr aModel,
        const Transform3D<>& wTa,
        ProximityModel::Ptr bModel,
        const Transform3D<>& wTb,
        double tolerance,
        ProximityStrategyData &data
        )
{
    RW_WARN("1");
	QueryData qdata = initQuery(aModel,bModel,data);
	RW_WARN("1");
    BOOST_FOREACH(const BulletModel& ma, qdata.a->models) {
        BOOST_FOREACH(const BulletModel& mb, qdata.b->models) {
        	RW_WARN("1");
        	ma.model->setWorldTransform( makeBtTransform( wTa*ma.t3d ) );
        	RW_WARN("1");
        	mb.model->setWorldTransform( makeBtTransform( wTb*mb.t3d ) );
        	RW_WARN("1");
        	ContactCB cback;
        	std::cout << "Call contact detect!" << std::endl;
        	_cworld->contactPairTest(ma.model.get(), mb.model.get(), cback );

        	if(cback.contacts.size()>0)
        		return true;

        	//contactPointResult.

            //data.getCollisionData()._nrBVTests += qdata.cache->_toleranceResult.NumBVTests();
            //data.getCollisionData()._nrPrimTests += qdata.cache->_toleranceResult.NumTriTests();

            //if (qdata.cache->_toleranceResult.CloserThanTolerance() != 0){
            //    return true;
            //}
        }
    }

	return false;
}

bool ProximityStrategyBullet::inCollision(ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	ProximityStrategyData &pdata)
{
    QueryData qdata = initQuery(aModel,bModel,pdata);
    BOOST_FOREACH(const BulletModel& ma, qdata.a->models) {
        BOOST_FOREACH(const BulletModel& mb, qdata.b->models) {
        	ma.model->setWorldTransform( makeBtTransform( wTa*ma.t3d ) );
        	mb.model->setWorldTransform( makeBtTransform( wTb*mb.t3d ) );
        	ContactCB cback;
        	_cworld->contactPairTest(ma.model.get(), mb.model.get(), cback );
        	if(cback.contacts.size()>0)
        		return true;
        	//contactPointResult.

            //data.getCollisionData()._nrBVTests += qdata.cache->_toleranceResult.NumBVTests();
            //data.getCollisionData()._nrPrimTests += qdata.cache->_toleranceResult.NumTriTests();

            //if (qdata.cache->_toleranceResult.CloserThanTolerance() != 0){
            //    return true;
            //}
        }
    }

	return false;
}


DistanceResult& ProximityStrategyBullet::distance(
										ProximityModel::Ptr aModel,
										const Transform3D<>& wTa,
										ProximityModel::Ptr bModel,
										const Transform3D<>& wTb,
										ProximityStrategyData &data
										)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    DistanceResult &rwresult = data.getDistanceData();
    rwresult.distance = 100000;

    rwresult.a = aModel;
    rwresult.b = bModel;

    BOOST_FOREACH(const BulletModel& ma, qdata.a->models) {
        BOOST_FOREACH(const BulletModel& mb, qdata.b->models) {

        	btConvexShape *shapeA = dynamic_cast<btConvexShape*>(ma.model.get());
        	btConvexShape *shapeB = dynamic_cast<btConvexShape*>(mb.model.get());
        	if(shapeA==NULL || shapeB==NULL )
        		continue;

        	// test if the objects are btConvexShape
        	Transform3D<> wTaa = wTa*ma.t3d;
        	Transform3D<> wTbb = wTb*mb.t3d;
        	btTransform wTa = makeBtTransform( wTaa );
        	btTransform wTb = makeBtTransform( wTbb );
        	btVector3 guess = makeBtVector( (wTbb.P()-wTaa.P())/2+wTaa.P() );

        	btGjkEpaSolver2::sResults result;
        	btGjkEpaSolver2::Distance(shapeA,wTa,shapeB,wTb,guess,result);

        	if( result.distance < rwresult.distance)
        		rwresult.distance = result.distance;
        }
    }
    return rwresult;
}

MultiDistanceResult& ProximityStrategyBullet::distances(
	ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	double threshold,
	ProximityStrategyData &data)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    MultiDistanceResult &rwresult = data.getMultiDistanceData();
    rwresult.distance = _threshold;

    rwresult.a = aModel;
    rwresult.b = bModel;

    BOOST_FOREACH(const BulletModel& ma, qdata.a->models) {
        BOOST_FOREACH(const BulletModel& mb, qdata.b->models) {
        }
    }
    return rwresult;
} 


 


std::vector<std::string> ProximityStrategyBullet::getGeometryIDs(rw::proximity::ProximityModel* model){
	std::vector<std::string> res;
	ProximityModelBullet *pmodel = (ProximityModelBullet*) model;
    BOOST_FOREACH(BulletModel &m, pmodel->models){
		res.push_back(m.geoid);
    }
	return res;
}

DistanceResult& ProximityStrategyBullet::distance(
												 ProximityModel::Ptr aModel,
												 const Transform3D<>& wTa,
												 ProximityModel::Ptr bModel,
												 const Transform3D<>& wTb,
												 double threshold,
												 ProximityStrategyData &data)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

}

void ProximityStrategyBullet::clear()
{
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}

CollisionStrategy::Ptr ProximityStrategyBullet::make()
{
    return ownedPtr(new ProximityStrategyBullet);
}

//! @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
void ProximityStrategyBullet::getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
									  rw::proximity::ProximityStrategyData& data)
{
	// TODO: implement contact generation
	RW_THROW("NOT IMPLEMENTED YET!");
}


