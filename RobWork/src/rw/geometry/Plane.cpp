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

#include "Plane.hpp"

#include "PlainTriMesh.hpp"

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

Q Plane::getParameters() const { 
    Q q(4);
    q(0) = _normal(0);
    q(1) = _normal(1);
    q(2) = _normal(2);
    q(3) = _d;
    return q;
}

TriMesh::Ptr Plane::createMesh(int resolution, double size) const  {
    size *= 0.5; //Scale s.t. the real size becomes size.

	// we find 4 points on the plane and create 2 triangles that represent the plane

    //Vector3D<> point = normalize(_normal) * _d/_normal.norm2();
	
    Vector3D<> otho;
    // find the orthogonal basis of the plane, eg xy-axis
    // use an axis to generate one orthogonal vector 
    if(fabs(angle(_normal, Vector3D<>::x())) > 0.001 && fabs(angle(_normal, Vector3D<>::x())) < Pi-0.001) {
        otho = cross(_normal, Vector3D<>::x());
    } else if( fabs(angle(_normal, Vector3D<>::y())) > 0.001 && fabs(angle(_normal, Vector3D<>::y())) < Pi-0.001) {
        otho = cross(_normal, Vector3D<>::y());
    } else if( fabs(angle(_normal, Vector3D<>::z())) > 0.001 && fabs(angle(_normal, Vector3D<>::z())) < Pi-0.001){
        otho = cross(_normal, Vector3D<>::z());
    }
    otho = normalize(otho);
    // and the final axis is then
    Vector3D<> ortho2 = normalize(cross(_normal, otho));


    Transform3D<> trans(-_normal*_d, Rotation3D<>(otho, ortho2, _normal));

    // now we generate the points in the two triangles
    rw::geometry::PlainTriMesh<>::Ptr mesh = ownedPtr( new rw::geometry::PlainTriMesh<>(2) );
    (*mesh)[0][0] = trans * (Vector3D<>::x()* size + Vector3D<>::y()* size);
    (*mesh)[0][1] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()* size);
    (*mesh)[0][2] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()*-size);

    (*mesh)[1][0] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()*-size);
    (*mesh)[1][1] = trans * (Vector3D<>::x()* size + Vector3D<>::y()*-size);
    (*mesh)[1][2] = trans * (Vector3D<>::x()* size + Vector3D<>::y()* size);

    return mesh;

}


TriMesh::Ptr Plane::createMesh(int resolution) const {
	return createMesh(resolution, 100 /* Default plane size */);
}
