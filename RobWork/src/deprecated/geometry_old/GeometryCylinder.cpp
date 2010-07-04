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


#include "GeometryCylinder.hpp"

#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>

using namespace rw::geometry;
using namespace rw::math;

namespace {
	std::string toString(float dx, float dy, int dz)
    {
        if (dz < 0) RW_THROW("Negative number of faces " << dz);

		std::stringstream str;
		str << "Cylinder " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

GeometryCylinder::GeometryCylinder(float radius, float height, int level) :
	Geometry(toString(radius, height, level) )
{
    float z = height/2.0f;

    for (int i = 0; i < level; i++) {
        //Construct Triangles for curved surface
        float x1 = (float)(radius * cos(i * 2 * Pi/level));
        float y1 = (float)(radius * sin(i * 2 * Pi/level));

        float x2 = (float)(radius * cos((i+1) * 2 * Pi/level));
        float y2 = (float)(radius * sin((i+1) * 2 * Pi/level));

        Vector3D<float> p1(x1, y1, z);
        Vector3D<float> p2(x1, y1, -z);
        Vector3D<float> p3(x2, y2, z);
        Vector3D<float> p4(x2, y2, -z);

        _faces.push_back(Face<float>(p1, p2, p3));
        _faces.push_back(Face<float>(p2, p4, p3));

        //Construct triangles for the end-plates
        _faces.push_back(
            Face<float>(
                Vector3D<float>(x1, y1, z),
                Vector3D<float>(x2, y2, z),
                Vector3D<float>(0,0,z)));

        _faces.push_back(
            Face<float>(
                Vector3D<float>(0,0,-z),
                Vector3D<float>(x2, y2, -z),
                Vector3D<float>(x1, y1, -z)));
    }
}

GeometryCylinder::~GeometryCylinder()
{}

const std::vector<Face<float> >& GeometryCylinder::getFaces() const
{
    return _faces;
}
