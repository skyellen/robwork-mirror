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

#include "ProximityStrategyYaobi.hpp"

#include <float.h>
#include <vector>

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>
#include <rw/models/Accessor.hpp>

#include <boost/foreach.hpp>

#include <yaobi/yaobi_mesh_interface.h>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

typedef std::pair<ProximityStrategyYaobi::ColModel,ProximityStrategyYaobi::ColModel> ModelPair;

//----------------------------------------------------------------------
// Utilities

namespace
{
    std::auto_ptr<yaobi::CollModel> makeModelFromSoup(
        const std::vector<Face<float> > &faceList)
    {
        unsigned char tri_stride(3);
        unsigned int num_tris(faceList.size());
        unsigned int num_verts(num_tris*3);
        yaobi::AppRealT *vertices = new yaobi::AppRealT[num_verts*3];
        int *tris = new int[num_tris*3];

        for(unsigned int triIdx=0,vertIdx=0;triIdx<num_tris;triIdx++,vertIdx+=3){
            Face<float> face = faceList[triIdx];
            for(size_t j=0;j<3;j++){
                vertices[vertIdx*3+0+j] = face._vertex1[j];
                vertices[vertIdx*3+3+j] = face._vertex2[j];
                vertices[vertIdx*3+6+j] = face._vertex3[j];
            }
            tris[vertIdx+0] = vertIdx+0;
            tris[vertIdx+1] = vertIdx+1;
            tris[vertIdx+2] = vertIdx+2;
        }

        yaobi::TriMeshInterface *tri = new yaobi::TriMeshInterface(
            num_verts, vertices,
            num_tris, tris,
            tri_stride, yaobi::OWN_DATA);
        std::auto_ptr<yaobi::CollModel> model(new yaobi::CollModel(tri, yaobi::OWN_DATA) );
        return model;
    }

    // Convert Transform3D to yaobi representation.
    void toTransform(
        const Transform3D<>& tr,
        yaobi::Real T[][4])
    {
        T[0][0] = tr(0,0); T[1][0] = tr(1,0); T[2][0] = tr(2,0);
        T[0][1] = tr(0,1); T[1][1] = tr(1,1); T[2][1] = tr(2,1);
        T[0][2] = tr(0,2); T[1][2] = tr(1,2); T[2][2] = tr(2,2);
        T[0][3] = tr(0,3); T[1][3] = tr(1,3); T[2][3] = tr(2,3);
    }

    std::auto_ptr<yaobi::CollModel> makeModel(const CollisionModelInfo& info)
    {
        typedef std::auto_ptr<yaobi::CollModel> T;
        if (info.getId() == "")
            return T(NULL);

        std::vector<Face<float> > faceList;
        try {
            if (FaceArrayFactory::getFaceArray(info.getId(), faceList)) {
            	return makeModelFromSoup(faceList);
            } else {
                RW_WARN("Can not obtain triangles from: " <<
                		StringUtil::quote(info.getId()));
            }
        }
        catch (const Exception& exp) {
            RW_WARN("Failed constructing collision model with message: "<<exp.getMessage().getText());
        }
        return T(NULL);;
    }

    void collide(
        const yaobi::CollModel &ma, const Transform3D<>& wTa,
        const yaobi::CollModel &mb, const Transform3D<>& wTb,
        bool firstContact,
        yaobi::CollideResult& result)
    {
        yaobi::Real ta[3][4], tb[3][4];

        toTransform(wTa, ta);
        toTransform(wTb, tb);

        yaobi::QueryType flag = firstContact ? yaobi::FIRST_CONTACT_ONLY : yaobi::ALL_CONTACTS;
        Collide(result, ta, ma, tb, mb, flag);
    }

    void createTestPairs(const ProximityStrategyYaobi::SharedModelList& m1s,
    					 const ProximityStrategyYaobi::SharedModelList& m2s,
    					 std::vector<ModelPair>& result ){
    	// test all m1s models against m2s models
    	BOOST_FOREACH(const ProximityStrategyYaobi::ColModel& m1, m1s){
    		BOOST_FOREACH(const ProximityStrategyYaobi::ColModel& m2, m2s){
    			result.push_back(ModelPair(m1,m2));
    		}
    	}
    }

}

//----------------------------------------------------------------------
// ProximityStrategyYaobi

ProximityStrategyYaobi::ProximityStrategyYaobi() {}

ProximityStrategyYaobi::~ProximityStrategyYaobi()
{

}

bool ProximityStrategyYaobi::hasModel(const Frame* frame) {
    typedef FrameModelMap::const_iterator I;
    I p = _frameModelMap.find(frame);
    if (p == _frameModelMap.end()) {
    	if( Accessor::collisionModelInfo().has(*frame) ) {
    		if( Accessor::collisionModelInfo().get(*frame).size() > 0 ){
    			return true;
    		}
        }
        return false;
    }
	return true;
}

bool ProximityStrategyYaobi::addModel(const Frame *frame,
									const std::vector< Face<float> > &faces)
{
	//Construct the new model and add it to the model list
    yaobi::CollModel *model = makeModelFromSoup(faces).release();

    if( model==NULL )
    	return false;
    // add it to the list so it will be cleaned up later
    SharedModel sharedModel(model);
    // update the modelMap
    _frameModelMap[frame].push_back( ColModel(Transform3D<>::identity(), sharedModel) );
    return true;
}

bool ProximityStrategyYaobi::addModel(const Frame* frame) {
	// TODO: check if models have already been added
	if( !Accessor::collisionModelInfo().has(*frame) )
		return false;
	std::vector<CollisionModelInfo> infos = Accessor::collisionModelInfo().get(*frame);
	BOOST_FOREACH(CollisionModelInfo &info, infos){
		if( _modelCache.isInCache(info.getId()) ){
			SharedModel sharedModel = _modelCache.get(info.getId());
			_frameModelMap[frame].push_back( ColModel(info.getTransform(),sharedModel) );
			continue;
		}
		yaobi::CollModel *yaobiModel = makeModel(info).release();
		if( yaobiModel==NULL )
			continue;
		_modelCache.add(info.getId(), yaobiModel);
		SharedModel sharedModel = _modelCache.get(info.getId());
		_frameModelMap[frame].push_back(ColModel(info.getTransform(),sharedModel));
	}
	return true;
}


const ProximityStrategyYaobi::SharedModelList& ProximityStrategyYaobi::getModels(const Frame* frame)
{
    // TODO: check model cache
     FrameModelMap::const_iterator p = _frameModelMap.find(frame);
    if (p == _frameModelMap.end()) {
        addModel(frame);
        p = _frameModelMap.find(frame);
    }
    RW_ASSERT(p != _frameModelMap.end());

    return p->second;
}

void ProximityStrategyYaobi::setFirstContact(bool b)
{
    _firstContact = b;
}

bool ProximityStrategyYaobi::inCollision(
    const Frame* a,
    const Transform3D<>& wTa,
    const Frame *b,
    const Transform3D<>& wTb)
{
    const SharedModelList& modelsA = getModels(a);
    if (modelsA.size()==0) return false;

    const SharedModelList& modelsB = getModels(b);
    if (modelsB.size()==0) return false;

    std::vector<ModelPair> testSet;
    createTestPairs(modelsA,modelsB,testSet);
    bool colliding = false;
    BOOST_FOREACH(ModelPair& pair, testSet){
    	yaobi::CollideResult result;
    	collide(*pair.first.second, wTa * pair.first.first,
    	        *pair.second.second, wTb * pair.second.first,
    	        _firstContact, result);
    	colliding |= result.IsColliding();
    }
    return colliding;
}

void ProximityStrategyYaobi::clear() {
	// TODO: also clear cache
    _frameModelMap.clear();
}

