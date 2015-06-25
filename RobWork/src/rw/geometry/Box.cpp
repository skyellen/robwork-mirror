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

#include "Box.hpp"

#include <rw/common/Ptr.hpp>
#include "PlainTriMesh.hpp"

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

namespace
{
	std::string toString(double dx, double dy, double dz){
		std::stringstream str;
		str << "Box " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

Box::Box(const rw::math::Q& initQ){
	setParameters(initQ);
}

Box::Box(double dx, double dy, double dz):
		_dx(dx),_dy(dy),_dz(dz)
{
}

Box::~Box() {
}


TriMesh::Ptr Box::createMesh(int resolution) const{
	PlainTriMeshF *mesh = new PlainTriMeshF(12);

	float x = (float)(_dx/2.0);
    float y = (float)(_dy/2.0);
    float z = (float)(_dz/2.0);

    Vector3D<float> p1(x, y, z);
    Vector3D<float> p2(x, y, -z);
    Vector3D<float> p3(-x, y, -z);
    Vector3D<float> p4(-x, y, z);

    Vector3D<float> p5(x, -y, z);
    Vector3D<float> p6(x, -y, -z);
    Vector3D<float> p7(-x, -y, -z);
    Vector3D<float> p8(-x, -y, z);

	(*mesh)[0] = Triangle<float>(p1,p2,p3);
	(*mesh)[1] = Triangle<float>(p3,p4,p1);

	(*mesh)[2] = Triangle<float>(p1,p5,p6);
	(*mesh)[3] = Triangle<float>(p6,p2,p1);
    //_faces.push_back(Face<float>(p1, p5, p6));
    //_faces.push_back(Face<float>(p6, p2, p1));

	(*mesh)[4] = Triangle<float>(p3,p2,p6);
	(*mesh)[5] = Triangle<float>(p6,p7,p3);
    //_faces.push_back(Face<float>(p3, p2, p6));
    //_faces.push_back(Face<float>(p6, p7, p3));

	(*mesh)[6] = Triangle<float>(p5,p8,p7);
	(*mesh)[7] = Triangle<float>(p7,p6,p5);
    //_faces.push_back(Face<float>(p5, p8, p7));
    //_faces.push_back(Face<float>(p7, p6, p5));

	(*mesh)[8] = Triangle<float>(p1,p4,p8);
	(*mesh)[9] = Triangle<float>(p8,p5,p1);
    //_faces.push_back(Face<float>(p1, p4, p8));
    //_faces.push_back(Face<float>(p8, p5, p1));

	(*mesh)[10] = Triangle<float>(p4,p3,p7);
	(*mesh)[11] = Triangle<float>(p7,p8,p4);
    //_faces.push_back(Face<float>(p4, p3, p7));
    //_faces.push_back(Face<float>(p7, p8, p4));
	return ownedPtr( mesh );
}

rw::math::Q Box::getParameters() const {
	Q q(3);
	q(0) = _dx;
	q(1) = _dy;
	q(2) = _dz;
	return q;
}

void Box::setParameters(const rw::math::Q& q) {
	if(q.size()!=3) {
		RW_THROW("Size of parameter list must equal 3!");
	}
	
	_dx = q(0);
	_dy = q(1);
	_dz = q(2);
}

bool Box::doIsInside(const rw::math::Vector3D<>& point){
    if(fabs(point[0])<_dx && fabs(point[1])<_dy && fabs(point[2])<_dz)
        return true;
    return false;

}
