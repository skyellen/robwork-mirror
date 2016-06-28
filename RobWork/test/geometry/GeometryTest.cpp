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



#include "../TestSuiteConfig.hpp"

#include <rw/loaders.hpp>
#include <rw/geometry.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Timer.hpp>
#include <string>
#include <fstream>

using namespace boost::unit_test;

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::loaders;

#define PRINT_TYPE_SIZE(arg) BOOST_TEST_MESSAGE("sizeof(" \
		<< typeid(arg).name() << ") = " << sizeof(arg))

BOOST_AUTO_TEST_CASE( TriangleTypeSize ){
	PRINT_TYPE_SIZE(Triangle<float>);
	PRINT_TYPE_SIZE(Triangle<double>);
	PRINT_TYPE_SIZE(TriangleN1<float>);
	PRINT_TYPE_SIZE(TriangleN1<double>);
	PRINT_TYPE_SIZE(TriangleN3<float>);
	PRINT_TYPE_SIZE(TriangleN3<double>);

	PRINT_TYPE_SIZE(IndexedTriangle<uint16_t>);
	PRINT_TYPE_SIZE(IndexedTriangle<uint32_t>);
	PRINT_TYPE_SIZE(IndexedTriangleN1<uint16_t>);
	PRINT_TYPE_SIZE(IndexedTriangleN1<uint32_t>);
	PRINT_TYPE_SIZE(IndexedTriangleN3<uint16_t>);
	PRINT_TYPE_SIZE(IndexedTriangleN3<uint32_t>);
}
/*
BOOST_AUTO_TEST_CASE( ConvexHullTest ){
	GiftWrapHull3D hull;

	std::vector<Vector3D<> > vertices;
	vertices.push_back()

	hull.rebuild(vertices);
}
*/

BOOST_AUTO_TEST_CASE( TriMeshProfiling ){
    // first we load a TriangleMesh
	PlainTriMeshN1F::Ptr mesh;
	Timer timer;
	timer.resetAndResume();
	mesh = STLFile::load( testFilePath() + "geoms/FingerMid.stl" );
	BOOST_TEST_MESSAGE("STL load time: " << timer.getTime());
	BOOST_TEST_MESSAGE("Mesh size    : " << mesh->getSize());

	// now convert it to idx
	timer.resetAndResume();
	IndexedTriMeshN0<float, uint16_t>::Ptr imeshf16 = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float, uint16_t> >(*mesh);
	BOOST_TEST_MESSAGE("toIdxMesh float uint16 time: " << timer.getTime());

	timer.resetAndResume();
	IndexedTriMeshN0<float, uint32_t>::Ptr imeshf32 = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float, uint32_t> >(*mesh);
	BOOST_TEST_MESSAGE("toIdxMesh float uint32 time: " << timer.getTime());

	timer.resetAndResume();
	IndexedTriMeshN0<double, uint16_t>::Ptr imeshd16 = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<double, uint16_t> >(*mesh);
	BOOST_TEST_MESSAGE("toIdxMesh double uint32 time: " << timer.getTime());

	timer.resetAndResume();
	IndexedTriMeshN0<double, uint32_t>::Ptr imeshd32 = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<double, uint32_t> >(*mesh);
	BOOST_TEST_MESSAGE( "toIdxMesh double uint32 time: " << timer.getTime() );

	// now we want to test the performance of accesing elements in the indexed trimesh
	timer.resetAndResume();
	IndexedTriangle<uint32_t> tri;
	for(int j=0;j<100;j++)
		for(size_t i=0;i<imeshf16->getSize();i++){
			tri = imeshf16->getIndexedTriangle(i);
			tri = imeshf16->getIndexedTriangle(i);
			tri = imeshf16->getIndexedTriangle(i);
			tri = imeshf16->getIndexedTriangle(i);
		}
	BOOST_TEST_MESSAGE( "IndexedTriMeshN0::getIndexedTriangle float uint16 get indirect time: " << timer.getTime() );

	timer.resetAndResume();
	IndexedTriMesh<float>::Ptr imesh = imeshf16;
	for(int j=0;j<100;j++)
		for(size_t i=0;i<imesh->getSize();i++){
			tri = imesh->getIndexedTriangle(i);
			tri = imesh->getIndexedTriangle(i);
			tri = imesh->getIndexedTriangle(i);
			tri = imesh->getIndexedTriangle(i);
		}
	BOOST_TEST_MESSAGE( "IndexedTriMesh::getIndexedTriangle float uint16 get indirect time: " << timer.getTime() );

	timer.resetAndResume();
	IndexedTriangle<uint16_t> tri16;
	for(int j=0;j<100;j++)
		for(size_t i=0;i<imeshf16->getSize();i++){
			tri16 = (*imeshf16)[(int)i];
			tri16 = (*imeshf16)[(int)i];
			tri16 = (*imeshf16)[(int)i];
			tri16 = (*imeshf16)[(int)i];
		}
	BOOST_TEST_MESSAGE( "IndexedTriMeshN0::operator[] float uint16 get direct1 time: " << timer.getTime() );


	timer.resetAndResume();
	const std::vector<IndexedTriangle<uint16_t> > &tris = imeshf16->getTriangles();
	for(int j=0;j<100;j++)
		for(size_t i=0;i<imeshf16->getSize();i++){
			tri16 = tris[i];
			tri16 = tris[i];
			tri16 = tris[i];
			tri16 = tris[i];
		}
	BOOST_TEST_MESSAGE("IndexedTriMeshN0::getTriangles float uint16 get direct2 time: " << timer.getTime());

	timer.resetAndResume();
	const IndexedTriangle<uint16_t> *triarray = &tris[0];
	for(int j=0;j<100;j++)
		for(size_t i=0;i<imeshf16->getSize();i++){
			tri16 = triarray[i];
			tri16 = triarray[i];
			tri16 = triarray[i];
			tri16 = triarray[i];
		}
	BOOST_TEST_MESSAGE("Raw triangle array float uint16 get direct3 time: " << timer.getTime());

	// now test if the indexed data is the same as the plain
	const float epsilon = (float)1e-5;
	timer.resetAndResume();
	for(size_t i=0; i<mesh->getSize(); i++){
		//std::cout << MetricUtil::dist2(imeshf16->getVertex(i, V1), ((*mesh)[i])[0]) << "<" << epsilon << std::endl;
		BOOST_CHECK_LT(imeshf16->getIndexedTriangle(i)[0], imeshf16->getVertices().size());
	    BOOST_CHECK_LT(imeshf16->getIndexedTriangle(i)[1], imeshf16->getVertices().size());
        BOOST_CHECK_LT(imeshf16->getIndexedTriangle(i)[2], imeshf16->getVertices().size());

		BOOST_CHECK_SMALL(MetricUtil::dist2(imeshf16->getVertex(i, V1), ((*mesh)[i])[0]), epsilon);
		BOOST_CHECK_SMALL(MetricUtil::dist2(imeshf16->getVertex(i, V2), ((*mesh)[i])[1]), epsilon);
		BOOST_CHECK_SMALL(MetricUtil::dist2(imeshf16->getVertex(i, V3), ((*mesh)[i])[2]), epsilon);
	}
/*
	int arr_size = imeshf16->getVertices().size();
	float array[imeshf16->getVertices().size()*3];
	uint16_t indices(mesh->size()*3);
	for(int i=0; i<arr_size; i++){
	    array[i*3+0] = imeshf16->getVertices()[i](0);
	    array[i*3+1] = imeshf16->getVertices()[i](1);
	    array[i*3+2] = imeshf16->getVertices()[i](2);
	}

    for(int i=0; i<mesh->size(); i++){
        indices[i*3+0] = imeshf16->getIndexedTriangle(i)[0];
        indices[i*3+1] = imeshf16->getIndexedTriangle(i)[1];
        indices[i*3+2] = imeshf16->getIndexedTriangle(i)[2];
    }

    timer.resetAndResume();
    for(int j=0;j<100;j++)
        for(size_t i=0;i<imeshf16->getSize();i++){
            for(int xtmp=0;xtmp<4;xtmp++){
                tri16[0] = indices[i+0];
                tri16[1] = indices[i+1];
                tri16[2] = indices[i+2];
            }
        }

    BOOST_TEST_MESSAGE("Indexing flat array float uint16 time: " << timer.getTime());
*/
}

