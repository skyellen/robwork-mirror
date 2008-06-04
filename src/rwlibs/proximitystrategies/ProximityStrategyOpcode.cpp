/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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
#include "ProximityStrategyOpcode.hpp"

#include <rw/models/Accessor.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <opcode/StdAfx.h>
#include <opcode/opcode_port.h>

#include <rw/models/CollisionModelInfo.hpp>

#include <boost/foreach.hpp>

#include <iostream>

using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rwlibs::proximitystrategies;

ProximityStrategyOpcode::ProximityStrategyOpcode()
{
    _AABBTC = new Opcode::AABBTreeCollider();

    _AABBTC->SetFirstContact(true);
    _AABBTC->SetFullBoxBoxTest(true);
    _AABBTC->SetFullPrimBoxTest(true);
    _AABBTC->SetTemporalCoherence(false);

    RW_ASSERT(_AABBTC->ValidateSettings() == 0); // ??

    if (_AABBTC->ValidateSettings() != NULL) {
        RW_WARN("Opcode settings are wrong!!");
    }
}

ProximityStrategyOpcode::~ProximityStrategyOpcode()
{
    FrameModelMap::const_iterator it;
    for(it = _frameModelMap.begin(); it != _frameModelMap.end(); ++it){
        Opcode::Model* model = it->second;
        if(model != NULL){
            Opcode::MeshInterface* meshInterface =
                (Opcode::MeshInterface*)model->GetMeshInterface();
            RW_ASSERT(meshInterface->GetVerts() != NULL);
            RW_ASSERT(meshInterface->GetTris() != NULL);

            delete[] meshInterface->GetVerts();
            delete[] meshInterface->GetTris();
            delete meshInterface;
            delete model;
        }
    }

    delete _AABBTC;
}

// inherited from CDStrategy
void ProximityStrategyOpcode::setFirstContact(bool b)
{
    _AABBTC->SetFirstContact(b);
}

bool ProximityStrategyOpcode::hasModel(const Frame* frame)
{
    FrameModelMap::iterator p = _frameModelMap.find(frame);

    if (p == _frameModelMap.end()) {
        const std::vector<CollisionModelInfo>* model = Accessor::collisionModelInfo().getPtr(*frame);
        return model && !model->empty();
    }

    return (*p).second != NULL;
}

bool ProximityStrategyOpcode::addModel(
    const Frame* frame, const std::vector<Face<float> >& faces)
{
	IceMaths::IndexedTriangle* iTri =
        new IceMaths::IndexedTriangle[faces.size()];

    IceMaths::Point* points =
        new IceMaths::Point[faces.size()*3];

    for(size_t i=0;i<faces.size();i++){
        points[i*3+0].Set( faces.at(i)._vertex1 );
        points[i*3+1].Set( faces.at(i)._vertex2 );
        points[i*3+2].Set( faces.at(i)._vertex3 );
        iTri[i].mVRef[0] = i*3+0;
        iTri[i].mVRef[1] = i*3+1;
        iTri[i].mVRef[2] = i*3+2;
    }
    Opcode::MeshInterface* iMesh =
        new Opcode::MeshInterface();

    iMesh->SetNbTriangles(faces.size());
    iMesh->SetNbVertices(faces.size()*3);
    iMesh->SetPointers(iTri,points);

    Opcode::Model* model = new Opcode::Model();

    // 1) Initialize the creation structure
    Opcode::OPCODECREATE OPCC;
    // Surface data

    //!< Mesh interface (access to triangles & vertices) (*)
    OPCC.mIMesh = iMesh;

    // Tree building settings

    //!< Limit number of primitives / node. If limit is 1, build a complete tree
    //(2*N-1 nodes)
    OPCC.mSettings.mLimit = 1;
    OPCC.mSettings.mRules =
        Opcode::SPLIT_BEST_AXIS |
        Opcode::SPLIT_SPLATTER_POINTS |
        Opcode::SPLIT_GEOM_CENTER;

    //!< true => discard leaf nodes (else use a normal tree)
    OPCC.mNoLeaf = true;

    //!< true => quantize the tree (else use a normal tree)
    OPCC.mQuantized = false;

    //Debug

    //!< true => keep a copy of the original tree (debug purpose)
    OPCC.mKeepOriginal = false;

    //!< true => allows OPCODE to reorganize client arrays
    OPCC.mCanRemap = false;

    // 2) Build the model
    bool buildSucces = model->Build(OPCC);
    RW_ASSERT(buildSucces); // ??
    if(!buildSucces){
        RW_WARN("Build unsuccessfull.");
        delete[] iTri;
        delete[] points;
        delete iMesh;
        delete model;
        return false;
    }

    _frameModelMap[frame] = model;
    return true;

}

bool ProximityStrategyOpcode::addModel(const Frame* frame)
{
    if (!Accessor::collisionModelInfo().has(*frame)) {
        _frameModelMap[frame] = NULL;
        return false;
    }

    std::vector<CollisionModelInfo> geomodels =
        Accessor::collisionModelInfo().get(*frame);

    if (geomodels.size() == 0) return true;

    BOOST_FOREACH(CollisionModelInfo &model, geomodels){
	    std::vector< Face<float> > faces;
	    try {
	        if (!FaceArrayFactory::getFaceArray(model.getId(), faces))
            {
	            RW_WARN(
                    "Can not construct triangles from string: "
                    << StringUtil::quote(model.getId()));
	            return false;
	        }
	    }
	    catch (const Exception& exp) {
	        RW_WARN(
                "Failed constructing collision model with message: "
                <<exp.getMessage().getText());
	        return false;
	    }
	    if(!addModel(frame, faces))
	    	return false;
    }
    return true;
}

bool ProximityStrategyOpcode::inCollision(
    const Frame* a,
    const Transform3D<>& wTa,
    const Frame *b,
    const Transform3D<>& wTb)
{
    static Opcode::BVTCache _colCache;

    _colCache.Model0 = _colCache.Model1 = NULL;

    // Check if the geomodel of frame a and b is in the frame Model map
    FrameModelMap::iterator frameFindA = _frameModelMap.find(a);
    if( frameFindA == _frameModelMap.end() ){
        addModel(a);
        frameFindA = _frameModelMap.find(a);
    }
    _colCache.Model0 = frameFindA->second;

    if(_colCache.Model0==NULL )
        return false; // One or both frames does not own a collision model

    FrameModelMap::iterator frameFindB = _frameModelMap.find(b);
    if( frameFindB == _frameModelMap.end() ){
        addModel(b);
        frameFindB = _frameModelMap.find(b);
    }
    _colCache.Model1 = frameFindB->second;

    if(_colCache.Model1==NULL )
        return false; // One or both frames does not own a collision model

    // convert Transform3D to Matrix4x4
    IceMaths::Matrix4x4 A_world(
        wTa(0,0),wTa(1,0),wTa(2,0), 0.0f,
        wTa(0,1),wTa(1,1),wTa(2,1), 0.0f,
        wTa(0,2),wTa(1,2),wTa(2,2), 0.0f,
        wTa(0,3),wTa(1,3),wTa(2,3), 1.0f);
    IceMaths::Matrix4x4 B_world(
        wTb(0,0),wTb(1,0),wTb(2,0), 0.0f,
        wTb(0,1),wTb(1,1),wTb(2,1), 0.0f,
        wTb(0,2),wTb(1,2),wTb(2,2), 0.0f,
        wTb(0,3),wTb(1,3),wTb(2,3), 1.0f);

    /*
    Opcode::MeshInterface* mesh0 =
        (Opcode::MeshInterface*)_colCache.Model0->GetMeshInterface();
    Opcode::MeshInterface* mesh1 =
        (Opcode::MeshInterface*)_colCache.Model1->GetMeshInterface();
    */

    // Collision query
    bool IsOk = _AABBTC->Collide(_colCache, &A_world, &B_world);

    RW_ASSERT(IsOk == true); // ??

    // Get collision status => if true, objects overlap
    if(IsOk){
        int Status = _AABBTC->GetContactStatus();

        if(Status) {
            return true;
        }
    }
    return false;
}

void ProximityStrategyOpcode::clear() 
{
    _frameModelMap.clear();
}

void ProximityStrategyOpcode::clearFrame(const rw::kinematics::Frame* frame)
{
    _frameModelMap[frame] = 0;
}
