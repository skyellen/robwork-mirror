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

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
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

        PQP_Tolerance(&result, ra, ta, &ma, rb, tb, &mb, tolerance);
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

        PQP_Distance(&result, ra, ta, ma, rb, tb, mb, rel_err, abs_err);
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

        PQP_DistanceMultiThreshold(&result, threshold, ra, ta, ma, rb, tb, mb, rel_err, abs_err);
    }

    void createTestPairs(
        const ProximityStrategyPQP::ModelList& m1s,
        const ProximityStrategyPQP::ModelList& m2s,
        std::vector<ProximityStrategyPQP::ModelPair>& result)
    {
    	// test all m1s models against m2s models
    	BOOST_FOREACH(const ProximityStrategyPQP::ColModel& m1, m1s) {
    		BOOST_FOREACH(const ProximityStrategyPQP::ColModel& m2, m2s) {
    			result.push_back(ProximityStrategyPQP::ModelPair(m1, m2));
    		}
    	}
    }
}

//----------------------------------------------------------------------
// ProximityStrategyPQP

ProximityStrategyPQP::ProximityStrategyPQP() :
    _firstContact(true)
{
	clearStats();
}

bool ProximityStrategyPQP::hasModel(const Frame* frame)
{
    typedef FrameModelMap::const_iterator I;
    const I p = _frameModelMap.find(frame);
    if (p == _frameModelMap.end()) {
    	if (Accessor::collisionModelInfo().has(*frame)) {
    		if (!Accessor::collisionModelInfo().get(*frame).empty()) {
    			return true;
    		}
        }
        return false;
    }
	return true;
}

bool ProximityStrategyPQP::addModel(
    const Frame* frame,
    const std::vector< Face<float> > &faces)
{
    PQP_Model *pqpModel = makePQPModelFromSoup(faces).release();
    if (!pqpModel) return false;

    SharedModel model = ownedPtr(pqpModel);
    _frameModelMap[frame].push_back(ColModel(Transform3D<>::identity(),model));
    return true;
}

bool ProximityStrategyPQP::addModel(const Frame* frame)
{
	if (!Accessor::collisionModelInfo().has(*frame)) return false;

	std::vector<CollisionModelInfo> modelInfos = Accessor::collisionModelInfo().get(*frame);
	BOOST_FOREACH(CollisionModelInfo &info, modelInfos){
		if( _modelCache.isInCache(info.getId()) ){
			SharedModel model = _modelCache.get(info.getId());
			_frameModelMap[frame].push_back(ColModel(info.getTransform(),model));
			continue;
		}
		PQP_Model* pqpModel = makePQPModel(info).release();
		if( pqpModel==NULL )
			continue;
		_modelCache.add(info.getId(), pqpModel);
		SharedModel sharedModel = _modelCache.get(info.getId());
		_frameModelMap[frame].push_back(ColModel(info.getTransform(),sharedModel));
	}
	return true;
}

const ProximityStrategyPQP::ModelList& ProximityStrategyPQP::getPQPModels(const Frame* frame)
{
    // TODO: check model cache

    typedef FrameModelMap::const_iterator I;
    I p = _frameModelMap.find(frame);
    if (p == _frameModelMap.end()) {
        const bool ok = addModel(frame);
        if (!ok)
        	_frameModelMap[frame]; // Force the insertion of an empty list.
        p = _frameModelMap.find(frame);
    }

    RW_ASSERT(p != _frameModelMap.end());

    return p->second;
}

void ProximityStrategyPQP::setFirstContact(bool b)
{
    _firstContact = b;
}

bool ProximityStrategyPQP::inCollision(
    const Frame* a,
    const Transform3D<>& wTa,
    const Frame* b,
    const Transform3D<>& wTb,
    double tolerance)
{
    const ModelList& modelsA = getPQPModels(a);
    const ModelList& modelsB = getPQPModels(b);

    BOOST_FOREACH(const ColModel& ma, modelsA) {
        BOOST_FOREACH(const ColModel& mb, modelsB) {

            PQP_ToleranceResult result;
            pqpTolerance(
                *ma.second, wTa * ma.first,
                *mb.second, wTb * mb.first,
                tolerance,
                result);

            if (result.CloserThanTolerance() != 0){
                return true;
            }
        }
    }

	return false;
}

bool ProximityStrategyPQP::inCollision(const Frame* a,
									   const Transform3D<>& wTa,
									   const Frame* b,
									   const Transform3D<>& wTb)
{
    const ModelList& modelsA = getPQPModels(a);
    const ModelList& modelsB = getPQPModels(b);

	PQP::PQP_CollideResult result;
    BOOST_FOREACH(const ColModel& ma, modelsA) {
        BOOST_FOREACH(const ColModel& mb, modelsB) {
            pqpCollide(
                *ma.second, wTa * ma.first,
                *mb.second, wTb * mb.first,
                result,
                _firstContact);

            _numBVTests += result.NumBVTests();
            _numTriTests += result.NumTriTests();
            if (result.Colliding() != 0)
            	return true;
        }
    }

    return false;
}

bool ProximityStrategyPQP::distance(DistanceResult &rwresult,
									const Frame* a,
									const Transform3D<>& wTa,
									const Frame* b,
									const Transform3D<>& wTb,
									double rel_err,
									double abs_err)
{
    const ModelList& modelsA = getPQPModels(a);
    if (modelsA.empty())
    	return false;

    const ModelList& modelsB = getPQPModels(b);
    if (modelsB.empty())
    	return false;

    rwresult.distance = DBL_MAX;


    std::vector<ModelPair> testSet;
    createTestPairs(modelsA,modelsB,testSet);
	PQP::PQP_DistanceResult distResult;
    BOOST_FOREACH(ModelPair& pair, testSet) {
    	pqpDistance(
            pair.first.second.get(), wTa * pair.first.first,
            pair.second.second.get(), wTb * pair.second.first,
            rel_err, abs_err, distResult);
    }

    rwresult.distance = distResult.distance;
    rwresult.p1 = fromRapidVector(distResult.p1);
    rwresult.p2 = fromRapidVector(distResult.p2);

    rwresult.f1 = a;
    rwresult.f2 = b;

    return true;
}

bool ProximityStrategyPQP::getDistances(
      MultiDistanceResult &rwresult,
      const Frame* a,
      const Transform3D<>& wTa,
      const Frame* b,
      const Transform3D<>& wTb,
      double threshold,
      double rel_err,
      double abs_err)
{
    const ModelList& modelsA = getPQPModels(a);
    if (modelsA.empty()) return false;

    const ModelList& modelsB = getPQPModels(b);
    if (modelsB.empty()) return false;

    PQP_MultiDistanceResult result;

    std::vector<ModelPair> testSet;
    createTestPairs(modelsA,modelsB,testSet);
    BOOST_FOREACH(ModelPair& pair, testSet){
	    pqpMultiDistance(
            threshold,
            pair.first.second.get(), wTa * pair.first.first,
            pair.second.second.get(), wTb * pair.second.first,
            rel_err,
            abs_err,
            result);
    }

    typedef std::map<int, int> IdMap;
    IdMap idMap;

    for(size_t i=0; i<result.id1s.size(); i++){
    	double dist = result.distances[i];
    	int id = result.id1s[i];
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
    for(size_t i=0; i<result.id2s.size(); i++){
    	double dist = result.distances[i];
    	int id = result.id2s[i];
    	IdMap::iterator res = idMap1.find(id);
    	if( res == idMap1.end() ){
    		idMap1[id] = i;
    		continue;
    	}
    	if( result.distances[ (*res).second ] > dist ){
    		(*res).second = i;
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

    size_t vsize = idMap.size() + idMap1.size();
    rwresult.p1s.resize(vsize);
    rwresult.p2s.resize(vsize);
    rwresult.distances.resize(vsize);

    int i=0;
    for(IdMap::iterator it = idMap.begin();it != idMap.end(); ++it,i++){
    	int idx = (*it).second;
        rwresult.distances[i] = result.distances[idx];
        rwresult.p1s[i] = fromRapidVector(result.p1s[idx]);
        rwresult.p2s[i] = fromRapidVector(result.p2s[idx]);
    }
    for(IdMap::iterator it = idMap1.begin();it != idMap1.end(); ++it,i++){
    	int idx = (*it).second;
        rwresult.distances[i] = result.distances[idx];
        rwresult.p1s[i] = fromRapidVector(result.p1s[idx]);
        rwresult.p2s[i] = fromRapidVector(result.p2s[idx]);
    }
    rwresult.f1 = a;
    rwresult.f2 = b;
    return true;
}


void ProximityStrategyPQP::clear()
{
    _frameModelMap.clear();
}

void ProximityStrategyPQP::clearFrame(const rw::kinematics::Frame* frame)
{
    // The collection of geometries for the frame will really be set to the
    // empty list. This means (on purpose) that geometries specified in the
    // workcell description won't be lazily loaded later on.
    _frameModelMap[frame].clear();
}

CollisionStrategyPtr ProximityStrategyPQP::make()
{
    return ownedPtr(new ProximityStrategyPQP);
}
