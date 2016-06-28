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

#include "QHull3D.hpp"
#include "QHullND.hpp"

#include <rw/common/macros.hpp>
#include <float.h>

using namespace std;
using namespace rw::geometry;
using namespace boost::numeric;
// Contact pos, normal, hastighed, depth, idA, idB

void QHull3D::rebuild(const std::vector<rw::math::Vector3D<> >& vertices){
    using namespace rw::math;
    // convert the vertice array to an array of double
    double *vertArray = new double[vertices.size()*3];
    // copy all data into the vertArray
    for(size_t i=0;i<vertices.size();i++){
        const Vector3D<> &vnd = vertices[i];
        vertArray[i*3+0] = vnd[0];
        vertArray[i*3+1] = vnd[1];
        vertArray[i*3+2] = vnd[2];
    }
    // build the hull
    qhull::build(3, vertArray, vertices.size(), _vertiIdxs, _faceIdxs, _faceNormalsTmp, _faceOffsets);
    delete[] vertArray;
    std::vector<int> vertIdxMap(vertices.size());
    _hullVertices.resize(_vertiIdxs.size());
    for(size_t i=0;i<_vertiIdxs.size(); i++){
        _hullVertices[i] = vertices[_vertiIdxs[i]];
        vertIdxMap[ _vertiIdxs[i] ] = (int)i;
    }
    for(size_t i=0;i<_faceIdxs.size();i++){
        int tmp = _faceIdxs[i];
        _faceIdxs[i] = vertIdxMap[ tmp ];
    }

    _faceOffsets.resize(_faceIdxs.size()/3);
    _faceNormals.resize(_faceIdxs.size()/3);
    for(size_t i=0;i<_faceIdxs.size()/3; i++){
        _faceNormals[i][0] = _faceNormalsTmp[i*3+0];
        _faceNormals[i][1] = _faceNormalsTmp[i*3+1];
        _faceNormals[i][2] = _faceNormalsTmp[i*3+2];
    }
}


bool QHull3D::isInside(const rw::math::Vector3D<>& vertex){
    using namespace rw::math;
    //const static double EPSILON = 0.0000001;
    if( _faceIdxs.size()==0 ){
        //std::cout << "No Tris" << std::endl;
        return 0;
    }

    double minDist = DBL_MAX;
    for(size_t i=0; i<_faceIdxs.size()/3; i++){
        RW_ASSERT(_faceIdxs.size()> i*3);
        int faceVerticeIdx = _faceIdxs[i*3];
        //RW_ASSERT(faceVerticeIdx<(int)vertices.size());
        RW_ASSERT(faceVerticeIdx<(int)_hullVertices.size());
        RW_ASSERT(i<_faceNormals.size());
        double dist =  _faceOffsets[i] + dot(vertex, _faceNormals[i]);
        // dist will be negative if point is inside, and positive if point is outside
        minDist = std::min( -dist, minDist );
        if(minDist<0)
            return false;
    }

    return minDist>=0;
}


double QHull3D::getMinDistOutside(const rw::math::Vector3D<>& vertex){
    return -getMinDistInside(vertex);
}


double QHull3D::getMinDistInside(const rw::math::Vector3D<>& vertex){
    using namespace rw::math;
    if( _faceIdxs.size()==0 ){
        return 0;
    }
    double minDist = DBL_MAX;
    for(size_t i=0; i<_faceIdxs.size()/3; i++){
        RW_ASSERT(_faceIdxs.size()> i*3);
        RW_ASSERT(i<_faceNormals.size());
        double dist =  _faceOffsets[i] + dot(vertex, _faceNormals[i]);
        // dist will be negative if point is inside, and positive if point is outside
        minDist = std::min( -dist, minDist );
    }
    return minDist;
}

rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<double> >::Ptr QHull3D::toTriMesh(){
    using namespace rw::math;
    using namespace rw::geometry;
    PlainTriMesh<TriangleN1<double> >::Ptr mesh = rw::common::ownedPtr(new PlainTriMesh<TriangleN1<double> >());

    for(size_t i=0;i<_faceIdxs.size()/3; i++){
        Vector3D<> v1 = _hullVertices[ _faceIdxs[i*3+0] ];
        Vector3D<> v2 = _hullVertices[ _faceIdxs[i*3+1] ];
        Vector3D<> v3 = _hullVertices[ _faceIdxs[i*3+2] ];
        mesh->add(rw::geometry::TriangleN1<double>(v1,v2,v3));
    }
    return mesh;
}
