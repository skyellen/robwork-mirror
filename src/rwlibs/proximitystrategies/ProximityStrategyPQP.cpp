/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "ProximityStrategyPQP.hpp"

#include <float.h>
#include <vector>

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;

using namespace PQP;


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
                const Face<double> face = faceList.at(i);
                model->AddTri(face._vertex1, face._vertex2, face._vertex3, i);
            }
        }
        model->EndModel();

        return model;
    }

    // Convert Transform3D to rapid representation.
    void toRapidTransform(
        const Transform3D<>& tr,
        double R[3][3],
        double T[3])
    {
        R[0][0] = tr(0,0); R[1][1] = tr(1,1); R[2][2] = tr(2,2);
        R[0][1] = tr(0,1); R[1][0] = tr(1,0); R[2][0] = tr(2,0);
        R[0][2] = tr(0,2); R[1][2] = tr(1,2); R[2][1] = tr(2,1);
        T[0] = tr(0,3);    T[1] = tr(1,3);    T[2] = tr(2,3);
    }
    
    // Convert from rapid representation to Transform3D.
    Transform3D<double> fromRapidTransform(double R[3][3], double T[3])
    {
    	Transform3D<double> tr;
    	tr(0,0) = R[0][0]; tr(1,1) = R[1][1]; tr(2,2) = R[2][2];
    	tr(0,1) = R[0][1]; tr(1,0) = R[1][0]; tr(2,0) = R[2][0];
    	tr(0,2) = R[0][2]; tr(1,2) = R[1][2]; tr(2,1) = R[2][1];
    	tr(0,3) = T[0];    tr(1,3) = T[1];    tr(2,3) = T[2];
    	
    	return tr;
    }
    
    Vector3D<double> fromRapidVector(double T[3]) {
    	Vector3D<double> vec;
    	vec(0) = T[0];
    	vec(1) = T[1];
    	vec(2) = T[2];
    	return vec;
    }

    std::auto_ptr<PQP_Model> makePQPModel(const rw::kinematics::Frame *frame)
    {
        typedef std::auto_ptr<PQP_Model> T;
        

        if (frame->getPropertyMap().has("CollisionModelID")) {
            std::string model = frame->getPropertyMap().getValue<std::string>("CollisionModelID");
            if (model == "")
                return T(NULL);
            std::vector<Face<float> > faceList;
           
            try {
                if (FaceArrayFactory::GetFaceArray(model, faceList)) {
                    return makePQPModelFromSoup(faceList);
                } else {
                    RW_WARN("Can not obtain triangles from: " << StringUtil::Quote(model));
                } 
            }
            catch (const Exception& exp) {
                RW_WARN("Failed constructing collision model with message: "<<exp.getMessage().getText());
            }           
            return T(NULL);
        } else
            return T(NULL);
    }

    void pqpCollide(
        PQP_Model* ma, const Transform3D<>& wTa,
        PQP_Model* mb, const Transform3D<>& wTb,
        bool firstContact,
        PQP_CollideResult& result)
    {
        double ra[3][3], rb[3][3], ta[3], tb[3];
        
        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);
        
        const int flag = firstContact ? PQP_FIRST_CONTACT : PQP_ALL_CONTACTS;
        PQP_Collide(&result, ra, ta, ma, rb, tb, mb, flag);
    }    
    
    void pqpTolerance(
        PQP_Model* ma, const Transform3D<>& wTa,
        PQP_Model* mb, const Transform3D<>& wTb,
        double tolerance,
        PQP_ToleranceResult& result)
    {
        double ra[3][3], rb[3][3], ta[3], tb[3];
        
        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);
        
        PQP_Tolerance(&result, ra, ta, ma, rb, tb, mb, tolerance);
    }
    
    void pqpDistance(
        PQP_Model* ma, const Transform3D<>& wTa,
        PQP_Model* mb, const Transform3D<>& wTb,
        double rel_err,
        double abs_err,
        PQP_DistanceResult& result)
    {
        double ra[3][3], rb[3][3], ta[3], tb[3];
        
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
        double ra[3][3], rb[3][3], ta[3], tb[3];
        
        toRapidTransform(wTa, ra, ta);
        toRapidTransform(wTb, rb, tb);
        
        PQP_DistanceMultiThreshold(&result, threshold, ra, ta, ma, rb, tb, mb, rel_err, abs_err);
    }
    
}

//----------------------------------------------------------------------
// ProximityStrategyPQP
namespace rwlibs { namespace proximitystrategies {

ProximityStrategyPQP::ProximityStrategyPQP() {}

ProximityStrategyPQP::~ProximityStrategyPQP()
{
    FrameModelMap::const_iterator it;
    for(it = _frameModelMap.begin(); it != _frameModelMap.end(); ++it) {
        PQP_Model* model = it->second;
        if(model != NULL){
            delete model;
        }
    }
}

bool ProximityStrategyPQP::hasModel(const Frame* frame) {
    typedef std::map<const Frame*, PQP_Model*>::const_iterator I;
    I p = _frameModelMap.find(frame);
    
    if (p == _frameModelMap.end()) {
        if (frame->getPropertyMap().has("CollisionModelID")) {
            std::string model = frame->getPropertyMap().getValue<std::string>("CollisionModelID");
            if (model != "")
                return true;
        }
        return false;
    }
    if ((*p).second != NULL)
        return true;
    else
        return false;
}

bool ProximityStrategyPQP::addModel(const Frame *frame,
									       const std::vector< Face<float> > &faces)
{

    //If something else exists, start by deleting it
    typedef std::map<const Frame*, PQP_Model*>::iterator I;
    I p = _frameModelMap.find(frame);
    if (p != _frameModelMap.end()) {
        _frameModelMap.erase(p);
        delete (*p).second;
    }

    //Construct the new model
    PQP_Model *model = makePQPModelFromSoup(faces).release();
    _frameModelMap[frame] = model;
    return model != NULL;
}

bool ProximityStrategyPQP::addModel(const Frame* frame) {
    PQP_Model* model = makePQPModel(frame).release();
    _frameModelMap[frame] = model;
    return model != NULL;
}


PQP_Model* ProximityStrategyPQP::getPQPModel(const Frame* frame)
{
    typedef std::map<const Frame*, PQP_Model*>::const_iterator I;

    I p = _frameModelMap.find(frame);
    if (p == _frameModelMap.end()) {
        addModel(frame);
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
    const Frame *b,
    const Transform3D<>& wTb,
    double tolerance)
{
    PQP_Model* modelA = getPQPModel(a);
    if (!modelA) return false; 

    PQP_Model* modelB = getPQPModel(b);
    if (!modelB) return false;

    PQP_ToleranceResult result;
    pqpTolerance(modelA, wTa, modelB, wTb, tolerance, result);
    return result.CloserThanTolerance()>0;
}

bool ProximityStrategyPQP::inCollision(
    const Frame* a,
    const Transform3D<>& wTa,
    const Frame *b,
    const Transform3D<>& wTb)
{
    PQP_Model* modelA = getPQPModel(a);
    if (!modelA) 
        return false; 

    PQP_Model* modelB = getPQPModel(b);
    if (!modelB) 
        return false;

    PQP_CollideResult result;
    pqpCollide(modelA, wTa, modelB, wTb, _firstContact, result);
    return result.Colliding() > 0;
}

bool ProximityStrategyPQP::distance(DistanceResult &rwresult,
                                    const Frame* a, 
                                    const Transform3D<>& wTa,
                                    const Frame* b, 
                                    const Transform3D<>& wTb,
       								double rel_err, double abs_err)
{
    rwresult.distance = DBL_MAX;
    PQP_Model* modelA = getPQPModel(a);
    if (!modelA) return false; 

    PQP_Model* modelB = getPQPModel(b);
    if (!modelB) return false;

    PQP_DistanceResult result;
    pqpDistance(modelA, wTa, modelB, wTb, rel_err, abs_err, result);

    rwresult.distance = result.distance;
    rwresult.p1 = fromRapidVector(result.p1);
    rwresult.p2 = fromRapidVector(result.p2);
    
    rwresult.f1 = a;
    rwresult.f2 = b;
    
    return true;
}

bool ProximityStrategyPQP::getDistances(
      MultiDistanceResult &rwresult,
      const Frame* a,
      const Transform3D<>& wTa,
      const Frame *b,
      const Transform3D<>& wTb,
      double threshold,
      double rel_err,
      double abs_err)
{
    PQP_Model* modelA = getPQPModel(a);
    if(!modelA) return false;
    
    PQP_Model* modelB = getPQPModel(b);
    if(!modelB) return false;

    PQP_MultiDistanceResult result;
    pqpMultiDistance(threshold, modelA, wTa, modelB, wTb, rel_err, abs_err, result);

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
    //std::cout << "SIZE OF MAP 0: " << idMap.size() << std::endl;

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


void ProximityStrategyPQP::clear() {
    _frameModelMap.clear();
}

}} // End of namespace
