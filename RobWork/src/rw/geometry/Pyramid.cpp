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

#include "Pyramid.hpp"

#include "PlainTriMesh.hpp"

using namespace rw::geometry;
using namespace rw::common;
using namespace rw::math;

 Pyramid::Pyramid(const rw::math::Q& initQ){
     setParameters(initQ);
 }

 Pyramid::Pyramid(double widthx, double widthy, double height)
     :_widthX(widthx),_widthY(widthy),_height(height)
 {

 }

 Pyramid::~Pyramid(){}

TriMesh::Ptr Pyramid::createMesh(int resolution) const{
    rw::geometry::PlainTriMeshF::Ptr mesh = ownedPtr( new rw::geometry::PlainTriMeshF( 6 ) );
    // the bottom two triangles
    (*mesh)[0].getVertex(0) = Vector3D<float>(static_cast<float>(-_widthX/2.0),static_cast<float>(-_widthY/2), 0);
    (*mesh)[0].getVertex(1) = Vector3D<float>(static_cast<float>(-_widthX/2.0), static_cast<float>((float)_widthY/2.0), 0);
    (*mesh)[0].getVertex(2) = Vector3D<float>(static_cast<float>(_widthX/2.0), static_cast<float>(_widthY/2), 0);

    (*mesh)[1].getVertex(0) = Vector3D<float>(static_cast<float>(_widthX/2.0),static_cast<float>( _widthY/2), 0);
    (*mesh)[1].getVertex(1) = Vector3D<float>(static_cast<float>(_widthX/2.0),static_cast<float>(-_widthY/2), 0);
    (*mesh)[1].getVertex(2) = Vector3D<float>(static_cast<float>(-_widthX/2.0),static_cast<float>(-_widthY/2.0), 0);

    // the four sides
    (*mesh)[2].getVertex(0) = Vector3D<float>( 0, 0, static_cast<float>(_height));
    (*mesh)[2].getVertex(1) = Vector3D<float>(static_cast<float>(_widthX/2.0),static_cast<float>(-_widthY/2.0), 0);
    (*mesh)[2].getVertex(2) = Vector3D<float>(static_cast<float>(_widthX/2.0),static_cast<float>(_widthY/2.0), 0);

    (*mesh)[3].getVertex(0) = Vector3D<float>( 0, 0, static_cast<float>(_height));
    (*mesh)[3].getVertex(1) = Vector3D<float>(static_cast<float>(_widthX/2.0), static_cast<float>(_widthY/2.0), 0);
    (*mesh)[3].getVertex(2) = Vector3D<float>(static_cast<float>(-_widthX/2.0), static_cast<float>(_widthY/2.0), 0);

    (*mesh)[4].getVertex(0) = Vector3D<float>( 0, 0, static_cast<float>(_height));
    (*mesh)[4].getVertex(1) = Vector3D<float>(static_cast<float>(-_widthX/2.0), static_cast<float>(_widthY/2.0), 0);
    (*mesh)[4].getVertex(2) = Vector3D<float>(static_cast<float>(-_widthX/2.0), static_cast<float>(-_widthY/2.0), 0);

    (*mesh)[5].getVertex(0) = Vector3D<float>( 0, 0, static_cast<float>(_height));
    (*mesh)[5].getVertex(1) = Vector3D<float>(static_cast<float>(-_widthX/2.0), static_cast<float>(-_widthY/2.0), 0);
    (*mesh)[5].getVertex(2) = Vector3D<float>(static_cast<float>( _widthX/2.0), static_cast<float>(-_widthY/2.0), 0);

    return mesh;
}

rw::math::Q Pyramid::getParameters() const{
    return rw::math::Q(3, _widthX, _widthY, _height);
}

void Pyramid::setParameters(const rw::math::Q& q) {
	if (q.size() != 3) {
		RW_THROW("Size of parameter list must equal 3!");
	}
	
	_widthX = q(0);
	_widthY = q(1);
	_height = q(2);
}

bool Pyramid::doIsInside(const rw::math::Vector3D<>& point){
    // test if point is on back of all faces of pyramid

    // first test bottom of pyramid
    if(point[2]<0)
        return false;

    // test side of line (widhtX/2,0)(0,_height) and (0,_height)(-widhtX/2,0)
    if( ((0 - _widthX/2)*(point[2] - 0) - (_height - 0)*( fabs(point[0]) - _widthX/2)) > 0 )
        return false;

    // test side of line (widhtY/2,0)(0,_height) and (0,_height)(-widhtY/2,0)
    if( ((0 - _widthY/2)*(point[2] - 0) - (_height - 0)*( fabs(point[1]) - _widthY/2)) > 0 )
        return false;

    // ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;

    return true;
}
