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


#include "GeometryBox.hpp"

using namespace rw::geometry;
using namespace rw::math;

namespace
{
	std::string toString(float dx, float dy, float dz){
		std::stringstream str;
		//str << "Box " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

GeometryBox::GeometryBox(float dx, float dy, float dz):
	Geometry( toString(dx,dy,dz) )
{
    float x = dx/2;
    float y = dy/2;
    float z = dz/2;

    Vector3D<float> p1(x, y, z);
    Vector3D<float> p2(x, y, -z);
    Vector3D<float> p3(-x, y, -z);
    Vector3D<float> p4(-x, y, z);

    Vector3D<float> p5(x, -y, z);
    Vector3D<float> p6(x, -y, -z);
    Vector3D<float> p7(-x, -y, -z);
    Vector3D<float> p8(-x, -y, z);

    _faces.push_back(Face<float>(p1, p2, p3));
    _faces.push_back(Face<float>(p3, p4, p1));

    _faces.push_back(Face<float>(p1, p5, p6));
    _faces.push_back(Face<float>(p6, p2, p1));

    _faces.push_back(Face<float>(p3, p2, p6));
    _faces.push_back(Face<float>(p6, p7, p3));

    _faces.push_back(Face<float>(p5, p8, p7));
    _faces.push_back(Face<float>(p7, p6, p5));

    _faces.push_back(Face<float>(p1, p4, p8));
    _faces.push_back(Face<float>(p8, p5, p1));

    _faces.push_back(Face<float>(p4, p3, p7));
    _faces.push_back(Face<float>(p7, p8, p4));
}

GeometryBox::~GeometryBox() {

}


const std::vector<Face<float> >& GeometryBox::getFaces() const {
    return _faces;
}
