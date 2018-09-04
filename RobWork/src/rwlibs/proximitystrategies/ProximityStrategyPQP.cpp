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


#include "ProximityStrategyPQP.hpp"

#include <PQP/PQP.h>

#include <float.h>
#include <vector>

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/geometry/IntersectUtil.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>



using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace PQP;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace
{

    // Convert Transform3D to rapid representation.
    void toRapidTransform(
        const Transform3D<>& tr,
        PQP_REAL R[3][3],
        PQP_REAL T[3])
    {
        R[0][0] = (PQP_REAL)tr.R()(0,0);
        R[0][1] = (PQP_REAL)tr.R()(0,1);
        R[0][2] = (PQP_REAL)tr.R()(0,2);
        R[1][0] = (PQP_REAL)tr.R()(1,0);
        R[1][1] = (PQP_REAL)tr.R()(1,1);
        R[1][2] = (PQP_REAL)tr.R()(1,2);
        R[2][0] = (PQP_REAL)tr.R()(2,0);
        R[2][1] = (PQP_REAL)tr.R()(2,1);
        R[2][2] = (PQP_REAL)tr.R()(2,2);

        T[0] = (PQP_REAL)tr.P()(0);
        T[1] = (PQP_REAL)tr.P()(1);
        T[2] = (PQP_REAL)tr.P()(2);
    }

    // Convert from rapid representation to Transform3D.
    Transform3D<> fromRapidTransform(PQP_REAL R[3][3], PQP_REAL T[3])
    {
    	Transform3D<> tr;
    	tr(0,0) = (double)R[0][0];
    	tr(2,2) = (double)R[2][2];
    	tr(0,1) = (double)R[0][1];
    	tr(1,0) = (double)R[1][0];
    	tr(2,0) = (double)R[2][0];
    	tr(0,2) = (double)R[0][2];
    	tr(1,2) = (double)R[1][2];
    	tr(2,1) = (double)R[2][1];
    	tr(0,3) = (double)T[0];
    	tr(1,3) = (double)T[1];
    	tr(2,3) = (double)T[2];
    	return tr;
    }

    Vector3D<> fromRapidVector(PQP_REAL T[3])
    {
        return Vector3D<>((double)T[0], (double)T[1], (double)T[2]);
    }

    void pqpCollide(
        PQP_Model& ma, const Transform3D<>& wTa,
        PQP_Model& mb, const Transform3D<>& wTb,
        PQP_CollideResult& result,
        bool firstContact)
    {
    	PQP_REAL ra[3][3], rb[3][3], ta[3], tb[3];

        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);
        int cflag = PQP_FIRST_CONTACT;
        if( !firstContact )
        	cflag = PQP_ALL_CONTACTS;
        PQP_Collide(&result, ra, ta, &ma, rb, tb, &mb, cflag);
    }

    void pqpTolerance(
        PQP_Model& ma, const Transform3D<>& wTa,
        PQP_Model& mb, const Transform3D<>& wTb,
        double tolerance,
        PQP_ToleranceResult& result)
    {
    	PQP_REAL ra[3][3], rb[3][3], ta[3], tb[3];

        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);

		PQP_Tolerance(&result, ra, ta, &ma, rb, tb, &mb, (PQP_REAL)tolerance);
    }

    void pqpDistance(
        PQP_Model* ma, const Transform3D<>& wTa,
        PQP_Model* mb, const Transform3D<>& wTb,
        double rel_err,
        double abs_err,
        PQP_DistanceResult& result)
    {
    	PQP_REAL ra[3][3], rb[3][3], ta[3], tb[3];

        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);

        PQP_Distance(&result, ra, ta, ma, rb, tb, mb, (PQP_REAL)rel_err, (PQP_REAL)abs_err);
    }

    void pqpMultiDistance(
        double threshold,
        PQP_Model* ma, const Transform3D<>& wTa,
        PQP_Model* mb, const Transform3D<>& wTb,
        double rel_err,
        double abs_err,
        PQP_MultiDistanceResult& result)
    {
    	PQP_REAL ra[3][3], rb[3][3], ta[3], tb[3];

        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);

        PQP_DistanceMultiThreshold(&result, (PQP_REAL)threshold, ra, ta, ma, rb, tb, mb, (PQP_REAL)rel_err, (PQP_REAL)abs_err);
    }

    void pqpDistanceThreshold(
		PQP_Model* ma, const Transform3D<>& wTa,
		PQP_Model* mb, const Transform3D<>& wTb,
		double threshold,
		double rel_err,
		double abs_err,
		PQP_DistanceResult& result)
	{
		PQP_REAL ra[3][3], rb[3][3], ta[3], tb[3];

		toRapidTransform(wTa, ra, ta);
		toRapidTransform(wTb, rb, tb);

		PQP_DistanceThreshold(&result, (PQP_REAL)threshold, ra, ta, ma, rb, tb, mb, (PQP_REAL)rel_err, (PQP_REAL)abs_err);
	}
}

//----------------------------------------------------------------------
// ProximityStrategyPQP

ProximityStrategyPQP::ProximityStrategyPQP() :
    //_firstContact(true),
	_threshold(DBL_MAX)
{
	clearStats();
}


rw::proximity::ProximityModel::Ptr ProximityStrategyPQP::createModel()
{
    PQPProximityModel *model = new PQPProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyPQP::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// remove models from cache
	const std::vector<std::string> geoms = model->getGeometryIDs();
	for(std::size_t i = 0; i < geoms.size(); i++) {
		removeGeometry(model,geoms[i]);
	}
}

bool ProximityStrategyPQP::addGeometry(rw::proximity::ProximityModel* model,
                                       const rw::geometry::Geometry& geom) {

    //std::cout << "add geometry: " << geom.getId() << std::endl;
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
    CacheKey key(geom.getGeometryData().get(),geom.getScale());
    if( _modelCache.has(key) ){
        std::cout << "Cached" << std::endl;
        pqpmodel = _modelCache.get(key);
    } else {
		TriMesh::Ptr mesh = gdata->getTriMesh(false);
        if(mesh->getSize()==0)
            return false;

        const double scale = geom.getScale();
        pqpmodel = ownedPtr(new PQP_Model());

        pqpmodel->BeginModel((int)mesh->getSize());
        {
            for (size_t i = 0; i < mesh->getSize(); i++) {
                // NB: Note the cast.
            	Triangle<double> face = mesh->getTriangle(i);
            	Vector3D<PQP_REAL> v0 = cast<PQP_REAL>(face[0]*scale);
            	Vector3D<PQP_REAL> v1 = cast<PQP_REAL>(face[1]*scale);
            	Vector3D<PQP_REAL> v2 = cast<PQP_REAL>(face[2]*scale);

                pqpmodel->AddTri(&v0[0], &v1[0], &v2[0], (int)i);
            }
        }
        pqpmodel->EndModel();
        _modelCache.add(key, pqpmodel);
    }

    RWPQPModel rwpqpmodel(geom.getId(), geom.getTransform(), pqpmodel);
    rwpqpmodel.ckey = key;
    pmodel->models.push_back( rwpqpmodel );

    _allmodels.push_back(pmodel->models.back());
    _geoIdToModelIdx[geom.getId()].push_back((int)_allmodels.size()-1);
    return true;
}

bool ProximityStrategyPQP::addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy){
    // we allways copy the data here
    return addGeometry(model,*geom);
}

bool ProximityStrategyPQP::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
    //std::cout << "Remove geometry: " << geomId << std::endl;
    PQPProximityModel *pmodel = (PQPProximityModel*) model;
    // remove from model
    int idx=-1;
    for(size_t i=0;i<pmodel->models.size();i++)
        if(pmodel->models[i].geoid==geomId){
            idx = (int)i;
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

void ProximityStrategyPQP::setFirstContact(bool b)
{
    RW_WARN("THIS METHOD ProximityStrategyPQP::setFirstContact IS DEPRECATED AND WILL HAVE NO EFFECT!");
	//_firstContact = b;
}

ProximityStrategyPQP::QueryData ProximityStrategyPQP::initQuery(ProximityModel::Ptr& aModel, ProximityModel::Ptr& bModel, ProximityStrategyData &data){
    QueryData qdata;
    if(data.getCache()==NULL || data.getCache()->_owner!=this)
        data.getCache() = ownedPtr( new PQPProximityCache(this));

    qdata.cache = static_cast<PQPProximityCache*>(data.getCache().get());

    qdata.a = (PQPProximityModel*)aModel.get();
    qdata.b = (PQPProximityModel*)bModel.get();
    return qdata;
}


bool ProximityStrategyPQP::doIsWithinDistance(
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

            data.getCollisionData()._nrBVTests += qdata.cache->_toleranceResult.NumBVTests();
            data.getCollisionData()._nrPrimTests += qdata.cache->_toleranceResult.NumTriTests();

            if (qdata.cache->_toleranceResult.CloserThanTolerance() != 0){
                return true;
            }
        }
    }

	return false;
}
/*
bool ProximityStrategyPQP::inCollision(ProximityModel::Ptr aModel,
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

bool ProximityStrategyPQP::doInCollision(ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	ProximityStrategyData &pdata)
{
    QueryData qdata = initQuery(aModel,bModel,pdata);

    CollisionResult &data = pdata.getCollisionData();
    data.clear();
	data.a = aModel;
	data.b = bModel;
	data._aTb = inverse(wTa)*wTb;

    size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    bool col_res = false;
    bool firstContact = pdata.getCollisionQueryType() == CollisionStrategy::FirstContact;

    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
            pqpCollide(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                qdata.cache->_collideResult,
                firstContact);

            data._nrBVTests += qdata.cache->_collideResult.NumBVTests();
            data._nrPrimTests += qdata.cache->_collideResult.NumTriTests();

            _numBVTests += qdata.cache->_collideResult.NumBVTests();
            _numTriTests += qdata.cache->_collideResult.NumTriTests();

            if (qdata.cache->_collideResult.Colliding() != 0){
            	//data._aTb = fromRapidTransform(qdata.cache->_collideResult.R,qdata.cache->_collideResult.T);

            	nrOfCollidingGeoms++;

            	// copy data to collision data res
           		data._collisionPairs.resize(nrOfCollidingGeoms);

            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxA = (int)geoIdxA;
            	data._collisionPairs[nrOfCollidingGeoms-1].geoIdxB = (int)geoIdxB;

            	int startIdx = (int)data._geomPrimIds.size();
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


DistanceResult& ProximityStrategyPQP::doDistance(
										ProximityModel::Ptr aModel,
										const Transform3D<>& wTa,
										ProximityModel::Ptr bModel,
										const Transform3D<>& wTb,
										ProximityStrategyData &data
										)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    DistanceResult &rwresult = data.getDistanceData();
    rwresult.distance = _threshold;

    rwresult.a = aModel;
    rwresult.b = bModel;

    int geoA = -1;
    int geoB = -1;
    BOOST_FOREACH(const RWPQPModel& ma, qdata.a->models) {
    	geoA++;
        BOOST_FOREACH(const RWPQPModel& mb, qdata.b->models) {
        	geoB++;
            pqpDistance(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                data.rel_err, data.abs_err, qdata.cache->_distResult);

            if(rwresult.distance>qdata.cache->_distResult.distance){
                rwresult.distance = qdata.cache->_distResult.distance;
                rwresult.p1 = ma.t3d*fromRapidVector(qdata.cache->_distResult.p1);
                rwresult.p2 = inverse(wTa)*wTb*mb.t3d*fromRapidVector(qdata.cache->_distResult.p2);

                rwresult.geoIdxA = geoA;
                rwresult.geoIdxB = geoB;
                rwresult.idx1 = ma.pqpmodel->last_tri->id;
                rwresult.idx2 = mb.pqpmodel->last_tri->id;
            }
        }
    }
    return rwresult;
}

MultiDistanceResult& ProximityStrategyPQP::doDistances(
	ProximityModel::Ptr aModel,
	const Transform3D<>& wTa,
	ProximityModel::Ptr bModel,
	const Transform3D<>& wTb,
	double threshold,
	ProximityStrategyData &data)
{
    QueryData qdata = initQuery(aModel,bModel,data);

    MultiDistanceResult &rwresult = data.getMultiDistanceData();
    rwresult.clear();
    rwresult.distance = _threshold;

    rwresult.a = aModel;
    rwresult.b = bModel;

    PQP_MultiDistanceResult &result = qdata.cache->_multiDistResult;
    result.clear();

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

            for(size_t i = 0; i<result.id1s.size(); i++){
                double dist = result.distances[i];
                int id = result.id1s[i];
                if (dist < rwresult.distance) {
                	rwresult.distance = dist;
                	rwresult.p1 = ma.t3d*fromRapidVector(result.p1s[i]);
                	rwresult.p2 = ma.t3d*fromRapidVector(result.p2s[i]);
                }
                IdMap::iterator res = idMap.find(id);
                if( res == idMap.end() ){
                    idMap[id] = (int)i;
                    continue;
                }
                if( result.distances[ (*res).second ] > dist ){
                    (*res).second = (int)i;
                }
            }

            IdMap idMap1;
            for(size_t j=0; j<result.id2s.size(); j++){
                double dist = result.distances[j];
                int id = result.id2s[j];
                IdMap::iterator res = idMap1.find(id);
                if( res == idMap1.end() ){
                    idMap1[id] = (int)j;
                    continue;
                }
                if( result.distances[ (*res).second ] > dist ){
                    (*res).second = (int)j;
                }
            }

            size_t prevSize = rwresult.p1s.size();

            size_t vsize = idMap.size() + idMap1.size();
            rwresult.p1s.resize(prevSize+vsize);
            rwresult.p2s.resize(prevSize+vsize);
            rwresult.distances.resize(prevSize+vsize);
            rwresult.p1prims.resize(prevSize+vsize);
            rwresult.p2prims.resize(prevSize+vsize);

            size_t k = prevSize;
            for(IdMap::iterator it = idMap.begin();it != idMap.end(); ++it,k++){
            	int idx = (*it).second;
                rwresult.distances[k] = result.distances[idx];
                rwresult.p1s[k] = ma.t3d*fromRapidVector(result.p1s[idx]);
                rwresult.p2s[k] = ma.t3d*fromRapidVector(result.p2s[idx]);
                rwresult.p1prims[k] = result.id1s[idx];
                rwresult.p2prims[k] = result.id2s[idx];
            }
            for(IdMap::iterator it = idMap1.begin();it != idMap1.end(); ++it,k++){
                int idx = (*it).second;
                rwresult.distances[k] = result.distances[idx];
                rwresult.p1s[k] = ma.t3d*fromRapidVector(result.p1s[idx]);
                rwresult.p2s[k] = ma.t3d*fromRapidVector(result.p2s[idx]);
                rwresult.p1prims[k] = result.id1s[idx];
                rwresult.p2prims[k] = result.id2s[idx];
            }
        }
    }
    return rwresult;
} 


std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> >
	ProximityStrategyPQP::getSurfaceNormals(MultiDistanceResult& res, int idx)
{
	// get tris from pqp_models and compute triangle normal

	PQPProximityModel* a = (PQPProximityModel*)res.a.get();
	PQPProximityModel* b = (PQPProximityModel*)res.b.get();

	if(a->getGeometryIDs().size()>1 || b->getGeometryIDs().size()>1){
		RW_THROW(" multiple geoms on one frame is not supported for normal extraction yet!");
	}

	int p1id = res.p1prims[idx];
	int p2id = res.p2prims[idx];

	PQP::Tri* atri = &a->models[0].pqpmodel->tris[p1id];
	PQP::Tri* btri = &b->models[0].pqpmodel->tris[p2id];

	rw::math::Vector3D<> atri_p1 = fromRapidVector(atri->p1);
	rw::math::Vector3D<> atri_p2 = fromRapidVector(atri->p2);
	rw::math::Vector3D<> atri_p3 = fromRapidVector(atri->p3);

	rw::math::Vector3D<> btri_p1 = fromRapidVector(btri->p1);
	rw::math::Vector3D<> btri_p2 = fromRapidVector(btri->p2);
	rw::math::Vector3D<> btri_p3 = fromRapidVector(btri->p3);

	rw::math::Vector3D<> n_p1 = a->models[0].t3d.R() * cross(atri_p2-atri_p1, atri_p3-atri_p1);
	rw::math::Vector3D<> n_p2 = b->models[0].t3d.R() * cross(btri_p2-btri_p1, btri_p3-btri_p1);

	return std::make_pair(n_p1, n_p2);

}

 


std::vector<std::string> ProximityStrategyPQP::getGeometryIDs(rw::proximity::ProximityModel* model){
	std::vector<std::string> res;
	PQPProximityModel *pmodel = (PQPProximityModel*) model;
    BOOST_FOREACH(RWPQPModel &m, pmodel->models){
		res.push_back(m.geoid);
        /*if( m.geoid==geom.getId() ){
            removeGeometry( model, geom.getId() );
            break;
        }*/
    }
	return res;
}

DistanceResult& ProximityStrategyPQP::doDistanceThreshold(
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
    rwresult.a = aModel;
    rwresult.b = bModel;

	PQP::PQP_DistanceResult distResult;

    int geoA = -1;
    int geoB = -1;
    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
    	geoA++;
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {
        	geoB++;
        	pqpDistanceThreshold(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                threshold,
                data.rel_err, data.abs_err, distResult);

            if(rwresult.distance>distResult.distance){
                rwresult.distance = distResult.distance;
                rwresult.p1 = ma.t3d*fromRapidVector(distResult.p1);
                rwresult.p2 = inverse(wTa)*wTb*mb.t3d*fromRapidVector(distResult.p2);

                //rwresult.f1 = a;
                //rwresult.f2 = b;

                rwresult.geoIdxA = geoA;
                rwresult.geoIdxB = geoB;
                rwresult.idx1 = ma.pqpmodel->last_tri->id;
                rwresult.idx2 = mb.pqpmodel->last_tri->id;
            }
        }
    }
    return rwresult;
}

void ProximityStrategyPQP::clear()
{
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}

CollisionStrategy::Ptr ProximityStrategyPQP::make()
{
    return ownedPtr(new ProximityStrategyPQP);
}


void ProximityStrategyPQP::getCollisionContacts(
		std::vector<CollisionStrategy::Contact>& contacts, rw::proximity::ProximityStrategyData &data)
{

	CollisionStrategy::Result& res = data.getCollisionData();

	PQPProximityModel* a = (PQPProximityModel*)res.a.get();
	PQPProximityModel* b = (PQPProximityModel*)res.b.get();

	if(a->getGeometryIDs().size()>1 || b->getGeometryIDs().size()>1){
		RW_THROW(" multiple geoms on one frame is not supported for normal extraction yet!");
	}

	//typedef std::pair<int,int> IntPair;
	BOOST_FOREACH( CollisionStrategy::Result::CollisionPair &cpair, res._collisionPairs){
		PQPModelPtr &amodel = a->models[cpair.geoIdxA].pqpmodel;
		PQPModelPtr &bmodel = b->models[cpair.geoIdxB].pqpmodel;

		CollisionStrategy::Contact contact;
		for(int i=cpair.startIdx; i<cpair.startIdx+cpair.size; i++ ){
			std::pair<int,int> &primPair = res._geomPrimIds[i];
			PQP::Tri* atri = &amodel->tris[primPair.first];
			PQP::Tri* btri = &bmodel->tris[primPair.second];

			Triangle<> triA(fromRapidVector(atri->p1), fromRapidVector(atri->p2), fromRapidVector(atri->p3));
			Triangle<> triB(res._aTb*fromRapidVector(btri->p1), res._aTb*fromRapidVector(btri->p2), res._aTb*fromRapidVector(btri->p3));
			rw::math::Vector3D<> dst1,dst2;

			if( IntersectUtil::intersetPtTriTri(triA, triB, dst1, dst2) ){
				// add to result
				contact.normalA = triA.calcFaceNormal();
				contact.normalB = triB.calcFaceNormal();
				contact.point = dst1;
				contacts.push_back( contact );
				contact.point = dst2;
				contacts.push_back( contact );
			}
		}

	}
}
