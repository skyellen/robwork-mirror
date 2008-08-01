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

#include <cfloat>
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
            tri_stride,
            yaobi::OWN_DATA);

        std::auto_ptr<yaobi::CollModel> model(new yaobi::CollModel(tri, yaobi::OWN_DATA));
        return model;
    }

    // Convert Transform3D to yaobi representation.
    void toTransform(
        const Transform3D<>& tr,
        yaobi::Real T[3][4])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = static_cast<yaobi::Real>(tr(i, j));
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
                RW_WARN(
                    "Can not obtain triangles from: " <<
                    StringUtil::quote(info.getId()));
            }
        }
        catch (const Exception& exp) {
            RW_WARN(
                "Failed constructing collision model with message: "
                << exp.getMessage().getText());
        }
        return T(NULL);;
    }

    void collide(
        const yaobi::CollModel &ma, const Transform3D<>& wTa,
        const yaobi::CollModel &mb, const Transform3D<>& wTb,
        bool firstContact,
        yaobi::CollideResult& result)
    {
        yaobi::Real ta[3][4];
        yaobi::Real tb[3][4];

        toTransform(wTa, ta);
        toTransform(wTb, tb);

        yaobi::QueryType flag = firstContact ? yaobi::FIRST_CONTACT_ONLY : yaobi::ALL_CONTACTS;
        Collide(result, ta, ma, tb, mb, flag);
    }
}

//----------------------------------------------------------------------
// ProximityStrategyYaobi

ProximityStrategyYaobi::ProximityStrategyYaobi() {}
ProximityStrategyYaobi::~ProximityStrategyYaobi() {}

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

bool ProximityStrategyYaobi::addModel(
    const Frame *frame,
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

bool ProximityStrategyYaobi::addModel(const Frame* frame)
{
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

const ProximityStrategyYaobi::ColModelList&
ProximityStrategyYaobi::getModels(const Frame* frame)
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
    const ColModelList& modelsA = getModels(a);
    if (modelsA.size()==0) return false;

    const ColModelList& modelsB = getModels(b);
    if (modelsB.size()==0) return false;

    std::vector<ModelPair> testSet;
    BOOST_FOREACH(const ProximityStrategyYaobi::ColModel& ma, modelsA){
        BOOST_FOREACH(const ProximityStrategyYaobi::ColModel& mb, modelsB){

            yaobi::CollideResult result;
            collide(
                *ma.second,
                wTa * ma.first,
                *mb.second,
                wTb * mb.first,
                _firstContact,
                result);
            if (result.IsColliding()) return true;
        }
    }

    return false;
}

void ProximityStrategyYaobi::clear()
{
	// TODO: also clear cache
    _frameModelMap.clear();
}

void ProximityStrategyYaobi::clearFrame(const Frame* frame)
{
    _frameModelMap[frame].clear();
}

std::auto_ptr<CollisionStrategy> ProximityStrategyYaobi::make()
{
    typedef std::auto_ptr<CollisionStrategy> T;
    return T(new ProximityStrategyYaobi);
}
