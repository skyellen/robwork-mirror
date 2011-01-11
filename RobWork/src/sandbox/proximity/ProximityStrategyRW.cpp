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

#include <float.h>
#include <vector>

#include <rw/geometry/TriMesh.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>
#include <rw/models/Accessor.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// ProximityStrategyRW

ProximityStrategyRW::ProximityStrategyRW() :
    _firstContact(true)
{
	clearStats();
}


rw::proximity::ProximityModel::Ptr ProximityStrategyRW::createModel()
{
    PQPProximityModel *model = new PQPProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyRW::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// TODO: though the models should probably be removed from cache
}

bool ProximityStrategyRW::addGeometry(rw::proximity::ProximityModel* model,
                                       const rw::geometry::Geometry& geom) {
    PQPProximityModel *pmodel = (PQPProximityModel*) model;

	PQPModelPtr pqpmodel;
	GeometryData::Ptr gdata = geom.getGeometryData();

    // check if geomid is in model. remove it if it has
    BOOST_FOREACH(RWPQPModel &m, pmodel->models){
        if( m.geoid==geom.getId() ){
            removeGeometry( model, geom.getId() );
            break;
        }
    }

    // check if model is in
    CacheKey key(gdata.get(),geom.getScale());
    if( _modelCache.has(key) ){
        pqpmodel = _modelCache.get(key);
    } else {
		TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0)
            return false;

        const double scale = geom.getScale();
        pqpmodel = ownedPtr(new PQP_Model());

        pqpmodel->BeginModel(mesh->getSize());
        {
            for (size_t i = 0; i < mesh->getSize(); i++) {
                // NB: Note the cast.
            	Triangle<double> face = mesh->getTriangle(i);
            	Vector3D<PQP_REAL> v0 = cast<PQP_REAL>(face[0]*scale);
            	Vector3D<PQP_REAL> v1 = cast<PQP_REAL>(face[1]*scale);
            	Vector3D<PQP_REAL> v2 = cast<PQP_REAL>(face[2]*scale);

                pqpmodel->AddTri(&v0[0], &v1[0], &v2[0], i);
            }
        }
        pqpmodel->EndModel();
        _modelCache.add(key, pqpmodel);
    }
    RWPQPModel rwpqpmodel(geom.getId(), geom.getTransform(), pqpmodel);
    rwpqpmodel.ckey = key;
    pmodel->models.push_back( rwpqpmodel );

    _allmodels.push_back(pmodel->models.back());
    _geoIdToModelIdx[geom.getId()].push_back(_allmodels.size()-1);
    return true;
}

bool ProximityStrategyRW::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
    PQPProximityModel *pmodel = (PQPProximityModel*) model;
    // remove from model
    size_t idx=-1;
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
}

void ProximityStrategyRW::setFirstContact(bool b)
{
    _firstContact = b;
}

ProximityStrategyRW::QueryData ProximityStrategyRW::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
    QueryData qdata;
    if(data.getCache()==NULL || data.getCache()->_owner!=this)
        data.getCache() = ownedPtr( new PQPProximityCache(this));

    qdata.cache = static_cast<PQPProximityCache*>(data.getCache().get());

    qdata.a = (PQPProximityModel*)aModel.get();
    qdata.b = (PQPProximityModel*)bModel.get();
    return qdata;
}


bool ProximityStrategyRW::inCollision(
        ProximityModel::Ptr aModel,
        const Transform3D<>& wTa,
        ProximityModel::Ptr bModel,
        const Transform3D<>& wTb,
        double tolerance,
        ProximityStrategyData &data
        )
{
    QueryData qdata = initQuery(aModel,bModel,data);
    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
            pqpTolerance(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                tolerance,
                qdata.cache->_toleranceResult);

            if (qdata.cache->_toleranceResult.CloserThanTolerance() != 0){
                return true;
            }
        }
    }

	return false;
}
/*
bool ProximityStrategyRW::inCollision(ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	ProximityStrategyData &data)
{

    bool collides = false;
    bool firstcontact = pdata.getCollisionQueryType() == FirstContact;

    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
            pqpCollide(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                qdata.cache->_collideResult,
                _firstContact);

            _numBVTests += result.NumBVTests();
            _numTriTests += result.NumTriTests();
            if (result.Colliding() != 0){
            	collides = true;
            	// copy all results to col data record
            	if( firstContact )
            		return true;
            }
        }
    }

    return collides;
}
*/

bool ProximityStrategyRW::inCollision(ProximityModel::Ptr aModel,
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

    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
            pqpCollide(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                qdata.cache->_collideResult,
                firstContact);

            _numBVTests += qdata.cache->_collideResult.NumBVTests();
            _numTriTests += qdata.cache->_collideResult.NumTriTests();
            if (qdata.cache->_collideResult.Colliding() != 0){
            	data.a = aModel;
            	data.b = bModel;
            	data._aTb = fromRapidTransform(qdata.cache->_collideResult.R,qdata.cache->_collideResult.T);

            	nrOfCollidingGeoms++;

            	// copy data to collision data res
           		data._collisionPairs.resize(nrOfCollidingGeoms);

            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;

            	int startIdx = data._geomPrimIds.size();
            	int size = qdata.cache->_collideResult.num_pairs;
            	data._collisionPairs[nrOfCollidingGeoms-1].startIdx = startIdx;
            	data._collisionPairs[nrOfCollidingGeoms-1].size = size;

            	data._geomPrimIds.resize(startIdx+size);

            	for(int j=0;j<size;j++){
            		data._geomPrimIds[startIdx+j].first = qdata.cache->_collideResult.pairs[j].id1;
            		data._geomPrimIds[startIdx+j].second= qdata.cache->_collideResult.pairs[j].id2;
            	}
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


DistanceResult& ProximityStrategyRW::distance(
										ProximityModel::Ptr aModel,
										const Transform3D<>& wTa,
										ProximityModel::Ptr bModel,
										const Transform3D<>& wTb,
										ProximityStrategyData &data
										)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    DistanceResult &rwresult = data.getDistanceData();
    rwresult.distance = DBL_MAX;

    rwresult.a = aModel;
    rwresult.b = bModel;

    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {

            pqpDistance(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                data.rel_err, data.abs_err, qdata.cache->_distResult);

            if(rwresult.distance>qdata.cache->_distResult.distance){
                rwresult.distance = qdata.cache->_distResult.distance;
                rwresult.p1 = ma.t3d*fromRapidVector(qdata.cache->_distResult.p1);
                rwresult.p2 = mb.t3d*fromRapidVector(qdata.cache->_distResult.p2);

                rwresult.idx1 = ma.pqpmodel->last_tri->id;
                rwresult.idx2 = mb.pqpmodel->last_tri->id;
            }
        }
    }
    return rwresult;
}

MultiDistanceResult& ProximityStrategyRW::distances(
	ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	double threshold,
	ProximityStrategyData &data)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    MultiDistanceResult &rwresult = data.getMultiDistanceData();
    rwresult.distance = DBL_MAX;

    rwresult.a = aModel;
    rwresult.b = bModel;

    PQP_MultiDistanceResult &result = qdata.cache->_multiDistResult;

    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
            pqpMultiDistance(
                threshold,
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                data.rel_err,
                data.abs_err,
                result);

            typedef std::map<int, int> IdMap;
            IdMap idMap;

            for(size_t i=0; i<result.id1s.size(); i++){
                double dist = result.distances[i];
                int id = result.id1s[i];
                rwresult.distance = std::min(rwresult.distance, dist);
                IdMap::iterator res = idMap.find(id);
                if( res == idMap.end() ){
                    idMap[id] = i;
                    continue;
                }
                if( result.distances[ (*res).second ] > dist ){
                    (*res).second = i;
                }
            }

            IdMap idMap1;
            for(size_t j=0; j<result.id2s.size(); j++){
                double dist = result.distances[j];
                int id = result.id2s[j];
                IdMap::iterator res = idMap1.find(id);
                if( res == idMap1.end() ){
                    idMap1[id] = j;
                    continue;
                }
                if( result.distances[ (*res).second ] > dist ){
                    (*res).second = j;
                }
            }

            //std::cout << "SIZE OF MAP 1: " << idMap1.size() << std::endl;
            /*
            IdMap idMap2;
            for(IdMap::iterator it = idMap.begin();it != idMap.end(); ++it){
                int id1 = (*it).first;
                int idx = (*it).second;
                int id2 = result.id2s[idx];

                IdMap::iterator res = idMap2.find( id2 );
                if( res == idMap2.end() ){
                    idMap2[ id2 ] = idx;
                    continue;
                }
                int idx2 = (*res).second;
                if( result.distances[ idx2 ] > result.distances[ idx ] ){
                    (*res).second = idx;
                }
            }

            size_t vsize = idMap2.size();
            */

            size_t prevSize = rwresult.p1s.size();

            size_t vsize = idMap.size() + idMap1.size();
            rwresult.p1s.resize(prevSize+vsize);
            rwresult.p2s.resize(prevSize+vsize);
            rwresult.distances.resize(prevSize+vsize);


            size_t i = prevSize;
            for(IdMap::iterator it = idMap.begin();it != idMap.end(); ++it,i++){
                int idx = (*it).second;
                rwresult.distances[i] = result.distances[idx];
                rwresult.p1s[i] = ma.t3d*fromRapidVector(result.p1s[idx]);
                rwresult.p2s[i] = ma.t3d*fromRapidVector(result.p2s[idx]);
            }
            for(IdMap::iterator it = idMap1.begin();it != idMap1.end(); ++it,i++){
                int idx = (*it).second;
                rwresult.distances[i] = result.distances[idx];
                rwresult.p1s[i] = ma.t3d*fromRapidVector(result.p1s[idx]);
                rwresult.p2s[i] = ma.t3d*fromRapidVector(result.p2s[idx]);
            }
            //rwresult.f1 = a;
            //rwresult.f2 = b;
        }
    }
    return rwresult;
}


std::vector<std::string> ProximityStrategyRW::getGeometryIDs(rw::proximity::ProximityModel* model){
	return std::vector<std::string>();
}

DistanceResult& ProximityStrategyRW::distance(
												 ProximityModel::Ptr aModel,
												 const Transform3D<>& wTa,
												 ProximityModel::Ptr bModel,
												 const Transform3D<>& wTb,
												 double threshold,
												 ProximityStrategyData &data)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();

    DistanceResult &rwresult = data.getDistanceData();
    rwresult.distance = DBL_MAX;
	PQP::PQP_DistanceResult distResult;

    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {

        	pqpDistanceThreshold(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                threshold,
                data.rel_err, data.abs_err, distResult);

            if(rwresult.distance>distResult.distance){
                rwresult.distance = distResult.distance;
                rwresult.p1 = ma.t3d*fromRapidVector(distResult.p1);
                rwresult.p2 = mb.t3d*fromRapidVector(distResult.p2);

                //rwresult.f1 = a;
                //rwresult.f2 = b;

                rwresult.idx1 = ma.pqpmodel->last_tri->id;
                rwresult.idx2 = mb.pqpmodel->last_tri->id;
            }
        }
    }
    return rwresult;
}

void ProximityStrategyRW::clear()
{
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}

CollisionStrategy::Ptr ProximityStrategyRW::make()
{
    return ownedPtr(new ProximityStrategyRW);
}
