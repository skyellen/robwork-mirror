#include <boost/test/unit_test.hpp>

#include <iostream>
#include <opcode/StdAfx.h>
#include <opcode/opcode_port.h>
#include <vector>
#include <math.h>

#include <rw/geometry/GeometrySTL.hpp>
#include <rw/geometry/Face.hpp>

using namespace rw;

void OpcodeTest(){
    std::vector< geometry::Face<float> > faceList;
    geometry::GeometrySTL::ReadSTL("testfiles/cube.stlb", faceList);
    IceMaths::IndexedTriangle *iTri = new IceMaths::IndexedTriangle[faceList.size()];
    IceMaths::Point *points = new IceMaths::Point[faceList.size()*3];
    for(unsigned int i=0;i<faceList.size();i++){
        points[i*3+0].Set( faceList.at(i)._vertex1 );
        points[i*3+1].Set( faceList.at(i)._vertex2 );
        points[i*3+2].Set( faceList.at(i)._vertex3 );
        iTri[i].mVRef[0] = i*3+0;
        iTri[i].mVRef[1] = i*3+1;
        iTri[i].mVRef[2] = i*3+2;
    }

    Opcode::MeshInterface iMesh;

    iMesh.SetNbTriangles(faceList.size());
    iMesh.SetNbVertices(faceList.size());
    iMesh.SetPointers(iTri,points);

    Opcode::Model sample;
    Opcode::Model sample0;

    // 1) Initialize the creation structure
    Opcode::OPCODECREATE OPCC;
    // Surface data
    OPCC.mIMesh = &iMesh; //!< Mesh interface (access to triangles & vertices) (*)
    // Tree building settings
    OPCC.mSettings.mLimit = 1; //!< Limit number of primitives / node. If limit is 1, build a complete tree (2*N-1 nodes)
//    OPCC.mSettings.mRules = Opcode::SPLIT_BEST_AXIS; //!< Building/Splitting rules (a combination of SplittingRules flags)
    OPCC.mSettings.mRules = Opcode::SPLIT_BEST_AXIS | Opcode::SPLIT_SPLATTER_POINTS | Opcode::SPLIT_GEOM_CENTER;
    OPCC.mNoLeaf = true; //!< true => discard leaf nodes (else use a normal tree)
    OPCC.mQuantized = false;		//!< true => quantize the tree (else use a normal tree)
    //Debug
    OPCC.mKeepOriginal = false; //!< true => keep a copy of the original tree (debug purpose)
    OPCC.mCanRemap = false;		//!< true => allows OPCODE to reorganize client arrays
    // 2) Build the model

    bool status = sample.Build(OPCC);
    status = status && sample0.Build(OPCC);

    BOOST_REQUIRE(status);

    Opcode::AABBTreeCollider TC;
    TC.SetFirstContact(false);
    TC.SetFullBoxBoxTest(true);
    TC.SetFullPrimBoxTest(true);
    TC.SetTemporalCoherence(false);

    BOOST_REQUIRE(TC.ValidateSettings() == NULL);

    // Setup cache
    static Opcode::BVTCache ColCache;
    ColCache.Model0 = &sample;
    ColCache.Model1 = &sample;

    IceMaths::Matrix4x4 world0;
    IceMaths::Matrix4x4 world1;
    world0.Identity();
    world1.Identity();

    world0.SetTrans(0.0f,0.0f,0.01f);
    // Collision query
    bool IsOk = TC.Collide(ColCache,&world0,NULL);
    // Get collision status => if true, objects overlap
    BOOST_REQUIRE(IsOk);

    BOOST_CHECK(TC.GetContactStatus() != 0);

    world0.SetTrans(100.0f, 0.0f, 0.01f);
    IsOk = TC.Collide(ColCache,&world0, NULL);

    BOOST_REQUIRE(IsOk);

    BOOST_CHECK(TC.GetContactStatus() == 0);

	delete iTri;
	delete points;
}
