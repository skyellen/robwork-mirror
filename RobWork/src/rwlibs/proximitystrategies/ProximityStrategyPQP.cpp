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

using namespace PQP;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace
{
/*
    std::auto_ptr<PQP_Model> makePQPModelFromSoup(
        const std::vector<Face<float> > &faceList)
    {
        std::auto_ptr<PQP_Model> model(new PQP_Model());

        model->BeginModel(faceList.size());
        {
            for (int i = 0; i < (int)faceList.size(); i++) {
                // NB: Note the cast.
                const Face<PQP_REAL> face = faceList.at(i);
                model->AddTri(face._vertex1, face._vertex2, face._vertex3, i);
            }
        }
        model->EndModel();

        return model;
    }
*/
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
    	tr(1,1) = (double)R[1][1];
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
/*
    std::auto_ptr<PQP_Model> makePQPModel(const CollisionModelInfo& info)
    {
        typedef std::auto_ptr<PQP_Model> T;
        if (info.getId() == "")
            return T(NULL);

        std::vector<Face<float> > faceList;
        try {
            if (FaceArrayFactory::getFaceArray(info.getId(), faceList)) {
            	return makePQPModelFromSoup(faceList);
            } else {
                RW_WARN("Can not obtain triangles from: " <<
                		StringUtil::quote(info.getId()));
            }
        }
        catch (const Exception& exp) {
            RW_WARN(
                "Failed to construct collision model: "
                << exp.getMessage().getText());
        }
        return T(NULL);;
    }
*/
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
    _firstContact(true)
{
	clearStats();
}


rw::proximity::ProximityModelPtr ProximityStrategyPQP::createModel()
{
    PQPProximityModel *model = new PQPProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyPQP::destroyModel(rw::proximity::ProximityModel* model){
	// when model gets deleted it should cleanup itself
	// TODO: though the models should probably be removed from cache
}

bool ProximityStrategyPQP::addGeometry(rw::proximity::ProximityModel* model,
                                       const rw::geometry::Geometry& geom) {
    PQPProximityModel *pmodel = (PQPProximityModel*) model;

    PQPModelPtr pqpmodel;
    GeometryDataPtr gdata = geom.getGeometryData();

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
    	TriMeshPtr mesh = gdata->getTriMesh(false);
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

bool ProximityStrategyPQP::removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId){
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

void ProximityStrategyPQP::setFirstContact(bool b)
{
    _firstContact = b;
}

bool ProximityStrategyPQP::collides(
    ProximityModelPtr aModel,
    const Transform3D<>& wTa,
    ProximityModelPtr bModel,
    const Transform3D<>& wTb,
    double tolerance)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();

    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {

            PQP_ToleranceResult result;
            pqpTolerance(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                tolerance,
                result);

            if (result.CloserThanTolerance() != 0){
                return true;
            }
        }
    }

	return false;
}

bool ProximityStrategyPQP::collides(ProximityModelPtr aModel,
									   const Transform3D<>& wTa,
									   ProximityModelPtr bModel,
									   const Transform3D<>& wTb)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();
    bool collides = false;
    PQP::PQP_CollideResult result;
    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {
            pqpCollide(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                result,
                _firstContact);

            _numBVTests += result.NumBVTests();
            _numTriTests += result.NumTriTests();
            if (result.Colliding() != 0){
            	collides = true;
            	// copy all results to col data record
            	if( _firstContact )
            		return true;
            }
        }
    }

    return collides;
}

bool ProximityStrategyPQP::collides(ProximityModelPtr aModel,
									   const Transform3D<>& wTa,
									   ProximityModelPtr bModel,
									   const Transform3D<>& wTb,
									   CollisionData &data)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);
	if(data._cache==NULL)
		data._cache = ownedPtr(new PQPCollisionCache(this));
	PQPCollisionCache *cache = (PQPCollisionCache*)data._cache.get();

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();
    size_t nrOfCollidingGeoms = 0, geoIdxA=0, geoIdxB=0;
    bool col_res = false;
    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {
            pqpCollide(
                *ma.pqpmodel, wTa * ma.t3d,
                *mb.pqpmodel, wTb * mb.t3d,
                cache->result,
                _firstContact);

            _numBVTests += cache->result.NumBVTests();
            _numTriTests += cache->result.NumTriTests();
            if (cache->result.Colliding() != 0){
            	data.a = a;
            	data.b = b;
            	data._aTb = fromRapidTransform(cache->result.R,cache->result.T);
            	if(_firstContact)
            		return true;
            	nrOfCollidingGeoms++;
            	// copy data to collision data res
            	if(data._collidePairs.size()<nrOfCollidingGeoms)
            		data._collidePairs.resize(nrOfCollidingGeoms);
            	data._collidePairs[nrOfCollidingGeoms-1].geoIdxA = geoIdxA;
            	data._collidePairs[nrOfCollidingGeoms-1].geoIdxB = geoIdxB;

            	data._collidePairs[nrOfCollidingGeoms-1]._geomPrimIds.resize(cache->result.num_pairs);
            	for(int j=0;j<cache->result.num_pairs;j++){
            		data._collidePairs[nrOfCollidingGeoms-1]._geomPrimIds[j].first = cache->result.pairs[j].id1;
            		data._collidePairs[nrOfCollidingGeoms-1]._geomPrimIds[j].second = cache->result.pairs[j].id2;
            	}
            	col_res = true;
            }
            geoIdxB++;
        }
        geoIdxA++;
    }
    return col_res;
}


bool ProximityStrategyPQP::calcDistance(DistanceResult &rwresult,
									ProximityModelPtr aModel,
									const Transform3D<>& wTa,
									ProximityModelPtr bModel,
									const Transform3D<>& wTb,
									double rel_err,
									double abs_err)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();

    rwresult.distance = DBL_MAX;
	PQP::PQP_DistanceResult distResult;

    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {

            pqpDistance(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                rel_err, abs_err, distResult);

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
    return true;
}

bool ProximityStrategyPQP::calcDistances(
      MultiDistanceResult &rwresult,
      ProximityModelPtr aModel,
      const Transform3D<>& wTa,
      ProximityModelPtr bModel,
      const Transform3D<>& wTb,
      double threshold,
      double rel_err,
      double abs_err)
{
    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();

    // initialize min distance with a large value
    rwresult.distance = 10000000.0;

    PQP_MultiDistanceResult result;

    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {
            pqpMultiDistance(
                threshold,
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                rel_err,
                abs_err,
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
    return true;
}


std::vector<std::string> ProximityStrategyPQP::getGeometryIDs(rw::proximity::ProximityModel* model){
	return std::vector<std::string>();
}

bool ProximityStrategyPQP::calcDistanceThreshold(DistanceResult &rwresult,
									ProximityModelPtr aModel,
									const Transform3D<>& wTa,
									ProximityModelPtr bModel,
									const Transform3D<>& wTb,
									double threshold,
									double rel_err,
									double abs_err)
{
    //RW_ASSERT(aModel->owner==this);
    //RW_ASSERT(bModel->owner==this);

    PQPProximityModel *a = (PQPProximityModel*)aModel.get();
    PQPProximityModel *b = (PQPProximityModel*)bModel.get();

    rwresult.distance = DBL_MAX;
	PQP::PQP_DistanceResult distResult;

    BOOST_FOREACH(const RWPQPModel& ma, a->models) {
        BOOST_FOREACH(const RWPQPModel& mb, b->models) {

        	pqpDistanceThreshold(
                ma.pqpmodel.get(), wTa * ma.t3d,
                mb.pqpmodel.get(), wTb * mb.t3d,
                threshold,
                rel_err, abs_err, distResult);

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
    return true;
}

void ProximityStrategyPQP::clear()
{
    _modelCache.clear();
    _geoIdToModelIdx.clear();
    _allmodels.clear();

    clearFrames();
}

CollisionStrategyPtr ProximityStrategyPQP::make()
{
    return ownedPtr(new ProximityStrategyPQP);
}
