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


#include "ProximityStrategyRW.hpp"

#include "BVTreeFactory.hpp"
#include "BVTreeColliderFactory.hpp"

#include <rw/proximity/ProximityStrategyData.hpp>

#include <vector>
#include <rw/geometry/TriMesh.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rw::proximity;

//----------------------------------------------------------------------
// ProximityStrategyRW

ProximityStrategyRW::ProximityStrategyRW()
{
	clearStats();
}

rw::proximity::ProximityModel::Ptr ProximityStrategyRW::createModel()
{
    RWProximityModel *model = new RWProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyRW::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// TODO: though the models should probably be removed from cache

}

bool ProximityStrategyRW::addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy){
    return addGeometry(model, *geom);
}

bool ProximityStrategyRW::addGeometry(ProximityModel* model, const Geometry& geom) {


    //Model(std::string id, rw::math::Transform3D<> trans, rw::proximity::BinaryBVTree<rw::geometry::OBB<> >::Ptr obbtree):
    //            geoid(id),t3d(trans),tree(obbtree){}

    RWProximityModel *pmodel = (RWProximityModel*) model;


    // check if geomid is in model. remove it if it has
    BOOST_FOREACH(Model::Ptr &m, pmodel->models){
        if( m->geoid==geom.getId() ){
            removeGeometry( model, geom.getId() );
            break;
        }
    }

    Model::Ptr rwmodel;
    // check if model is in
    CacheKey key(geom.getId(),geom.getScale());
    if( _modelCache.has(key) ){
        rwmodel = _modelCache.get(key);
    } else {
        GeometryData::Ptr gdata = geom.getGeometryData();
        TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0)
            return false;

        // TODO: we don't use scale for now
        const double scale = geom.getScale();

        BVTreeFactory treefactory;

        //Timer t;
        //std::cout << "Mesh size: " << mesh->getSize() << std::endl;
        BinaryOBBPtrTreeD::Ptr tree = treefactory.makeTopDownOBBTreeCovarMedian<BinaryOBBPtrTreeD>(mesh,1);
        //std::cout << "Time to create OBB tree: " << t.toString("ss:zzz") << std::endl;
        rwmodel = ownedPtr( new Model(geom.getId(), geom.getTransform(), tree) );
        rwmodel->ckey = key;
        rwmodel->scale = scale;
        _modelCache.add(key, rwmodel);
    }

    pmodel->models.push_back( rwmodel );
    //_allmodels.push_back(pmodel->models.back());
    //_geoIdToModelIdx[geom.getId()].push_back(_allmodels.size()-1);
    return true;
}

bool ProximityStrategyRW::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
    RWProximityModel *pmodel = (RWProximityModel*) model;
    size_t idx=-1;
    for(size_t i=0;i<pmodel->models.size();i++)
        if(pmodel->models[i]->geoid==geomId){
            idx = i;
            break;
        }
    // if the geometry id was not found then it is not located in the model
    if(idx<0){
        //RW_THROW("No geometry with id: \""<< geomId << "\" exist in proximity model!");
        return false;
    }

    // it was found so we remove it from cache
    _modelCache.remove(pmodel->models[idx]->ckey);
    std::vector<Model::Ptr>::iterator iter = pmodel->models.begin();
    for(;iter!=pmodel->models.end();++iter){
        if((*iter)->geoid==geomId){
            pmodel->models.erase(iter);
            return true;
        }
    }
    return false;
}

ProximityStrategyRW::QueryData ProximityStrategyRW::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
    QueryData qdata;

    if(data.getCache()==NULL || data.getCache()->_owner!=this)
        data.getCache() = ownedPtr( new PCache(this));

    qdata.cache = static_cast<PCache*>(data.getCache().get());
    if(qdata.cache->tcollider==NULL)
        qdata.cache->tcollider = ownedPtr( BVTreeColliderFactory::makeBalancedDFSColliderOBB<BinaryOBBPtrTreeD>() );

    qdata.a = (RWProximityModel*)aModel.get();
    qdata.b = (RWProximityModel*)bModel.get();
    return qdata;
}


std::vector<std::string> ProximityStrategyRW::getGeometryIDs(rw::proximity::ProximityModel* model){
	std::vector<std::string> res;
	RWProximityModel *pmodel = (RWProximityModel*) model;
    BOOST_FOREACH(Model::Ptr &m, pmodel->models) {
		res.push_back(m->geoid);
    }
	return res;



	//return model->getGeometryIDs();
}

bool ProximityStrategyRW::doInCollision(ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	ProximityStrategyData &pdata)
{
    QueryData qdata = initQuery(aModel,bModel,pdata);

    CollisionResult &data = pdata.getCollisionData();

    data.clear();

    size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    bool col_res = false;
    bool firstContact = pdata.getCollisionQueryType() == FirstContact;

    qdata.cache->tcollider->setQueryType( pdata.getCollisionQueryType() );

    BOOST_FOREACH(Model::Ptr &ma, qdata.a->models) {
        BOOST_FOREACH(Model::Ptr &mb, qdata.b->models) {
            //int startIdx = data._geomPrimIds.size();
            bool res = qdata.cache->tcollider->collides(wTa*ma->t3d, *ma->tree, wTb*mb->t3d, *mb->tree, &data._geomPrimIds);

            //std::cout << res << std::endl;
            _numBVTests += qdata.cache->tcollider->getNrOfTestedBVs();
            _numTriTests += qdata.cache->tcollider->getNrOfTestedPrimitives();

            if(res==true){
                data.a = aModel;
                data.b = bModel;
                data._aTb = inverse(wTa)*wTb;
                nrOfCollidingGeoms++;


                //data._collisionPairs.resize(nrOfCollidingGeoms);
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;
                //data._collisionPairs[nrOfCollidingGeoms-1].startIdx = startIdx;
                //data._collisionPairs[nrOfCollidingGeoms-1].size = data._geomPrimIds.size()-startIdx;

                if(firstContact)
                    return true;
                col_res = true;
            }
            geoIdxB++;
        }
        geoIdxA++;
    }
    return col_res;
}

bool ProximityStrategyRW::doIsWithinDistance(ProximityModel::Ptr aModel,
    const Transform3D<>& wTa,
    ProximityModel::Ptr bModel,
    const Transform3D<>& wTb,
    double tolerance,
    ProximityStrategyData &pdata)
{
    QueryData qdata = initQuery(aModel,bModel,pdata);

    CollisionResult &data = pdata.getCollisionData();

    data.clear();

    size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    bool col_res = false;
    bool firstContact = pdata.getCollisionQueryType() == FirstContact;

    qdata.cache->tcollider->setQueryType( pdata.getCollisionQueryType() );

    BOOST_FOREACH(Model::Ptr &ma, qdata.a->models) {
        BOOST_FOREACH(Model::Ptr &mb, qdata.b->models) {
            //int startIdx = data._geomPrimIds.size();
            bool res = qdata.cache->tcollider->collides(wTa*ma->t3d, *ma->tree, wTb*mb->t3d, *mb->tree, &data._geomPrimIds);

            //std::cout << res << std::endl;
            _numBVTests += qdata.cache->tcollider->getNrOfTestedBVs();
            _numTriTests += qdata.cache->tcollider->getNrOfTestedPrimitives();

            if(res==true){
                data.a = aModel;
                data.b = bModel;
                data._aTb = inverse(wTa)*wTb;
                nrOfCollidingGeoms++;

                //data._collisionPairs.resize(nrOfCollidingGeoms);
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
                //data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;
                //data._collisionPairs[nrOfCollidingGeoms-1].startIdx = startIdx;
                //data._collisionPairs[nrOfCollidingGeoms-1].size = data._geomPrimIds.size()-startIdx;

                if(firstContact)
                    return true;
                col_res = true;
            }
            geoIdxB++;
        }
        geoIdxA++;
    }
    return col_res;
}
/*
namespace {

    void RWTestDistance(Model::Ptr &ma, rw::math::Transform3D<>& aT,
                                             Model::Ptr &mb, rw::math::Transform3D<>& bT,
                                             ProximityStrategyData& data)
    {

        // traverse the tree here


    }

}
*/

DistanceStrategy::Result& ProximityStrategyRW::doDistance(
    ProximityModel::Ptr a,
    const Transform3D<>& wTa,
    ProximityModel::Ptr b,
    const Transform3D<>& wTb,
    ProximityStrategyData& pdata)
{
    QueryData qdata = initQuery(a,b,pdata);

    DistanceResult &data = pdata.getDistanceData();

    data.clear();

    //size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    //bool col_res = false;
    //bool firstContact = pdata.getCollisionQueryType() == FirstContact;

    qdata.cache->tcollider->setQueryType( pdata.getCollisionQueryType() );

    //BOOST_FOREACH(Model::Ptr &ma, qdata.a->models) {
        //BOOST_FOREACH(Model::Ptr &mb, qdata.b->models) {
            //bool res = qdata.cache->tcollider->collides(wTa*ma->t3d, *ma->tree, wTb*mb->t3d, *mb->tree, &data._geomPrimIds);
            //RWTestDistance(wTa*ma->t3d, *ma->tree, wTb*mb->t3d, *mb->tree, data);


            //std::cout << res << std::endl;
            //_numBVTests += qdata.cache->tcollider->getNrOfTestedBVs();
            //_numTriTests += qdata.cache->tcollider->getNrOfTestedPrimitives();

            /*
            if(res==true){
                data.a = aModel;
                data.b = bModel;
                data._aTb = inverse(wTa)*wTb;
                nrOfCollidingGeoms++;

                if(firstContact)
                    return true;
                col_res = true;
            }
            */
            //geoIdxB++;
        //}
        //geoIdxA++;
    //}
    return data;
}


void ProximityStrategyRW::getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
											  ProximityStrategyData& data)
{
	RW_THROW("Not implemented yet!");
}


void ProximityStrategyRW::clear()
{
    _modelCache.clear();
    _allModels.clear();
    clearFrames();
}
