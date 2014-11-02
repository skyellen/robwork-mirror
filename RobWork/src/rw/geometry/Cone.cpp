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

#include "Cone.hpp"

#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>
#include "PlainTriMesh.hpp"
#include <rw/math/Vector2D.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

Cone::Cone(const rw::math::Q& initQ){
    _height=initQ(0);
    _radiusTop=initQ(1);
    _radiusBottom=initQ(2);
}

Cone::Cone(double height, double radiusTop, double radiusBot):
        _radiusTop(radiusTop), _radiusBottom(radiusBot), _height(height)
{

}


rw::math::Q Cone::getParameters() const{
    return Q::zero(3);
}


Cone::~Cone(){}

TriMesh::Ptr Cone::createMesh(int resolution) const{
	int level = resolution;
	if(resolution<0)
		level = 16; // default

	float z = (float)(_height/2.0f);

	if (_radiusTop <= 0 || _radiusBottom <= 0) {
		PlainTriMeshF *mesh = new PlainTriMeshF(2*level);
		if (_radiusTop <= 0) {
			for (int i = 0; i < level; i++) {
				//Construct Triangles for curved surface
				float x3 = (float)(_radiusBottom * cos(i * 2 * Pi/level));
				float y3 = (float)(_radiusBottom * sin(i * 2 * Pi/level));

				float x4 = (float)(_radiusBottom * cos((i+1) * 2 * Pi/level));
				float y4 = (float)(_radiusBottom * sin((i+1) * 2 * Pi/level));

				Vector3D<float> p1(0, 0, z);
				Vector3D<float> p3(x3, y3, -z);
				Vector3D<float> p4(x4, y4, -z);

				(*mesh)[i*2+0] = Triangle<float>(p1, p3, p4);

				//Construct triangles for the end-plates
				Vector3D<float> p6(0, 0, -z);
				(*mesh)[i*2+1] = Triangle<float>(p3, p6, p4);
			}
		} else {
			for (int i = 0; i < level; i++) {
				//Construct Triangles for curved surface
				float x1 = (float)(_radiusTop * cos(i * 2 * Pi/level));
				float y1 = (float)(_radiusTop * sin(i * 2 * Pi/level));

				float x2 = (float)(_radiusTop * cos((i+1) * 2 * Pi/level));
				float y2 = (float)(_radiusTop * sin((i+1) * 2 * Pi/level));

				Vector3D<float> p1(x1, y1, z);
				Vector3D<float> p2(x2, y2, z);
				Vector3D<float> p3(0, 0, -z);

				(*mesh)[i*2+0] = Triangle<float>(p1, p3, p2);

				//Construct triangles for the end-plates
				Vector3D<float> p5(0, 0,  z);
				(*mesh)[i*2+1] = Triangle<float>(p1, p2, p5);
			}
		}
		return ownedPtr(mesh);
	} else {
		PlainTriMeshF *mesh = new PlainTriMeshF(4*level);
		for (int i = 0; i < level; i++) {
			//Construct Triangles for curved surface
			float x1 = (float)(_radiusTop * cos(i * 2 * Pi/level));
			float y1 = (float)(_radiusTop * sin(i * 2 * Pi/level));

			float x2 = (float)(_radiusTop * cos((i+1) * 2 * Pi/level));
			float y2 = (float)(_radiusTop * sin((i+1) * 2 * Pi/level));

			float x3 = (float)(_radiusBottom * cos(i * 2 * Pi/level));
			float y3 = (float)(_radiusBottom * sin(i * 2 * Pi/level));

			float x4 = (float)(_radiusBottom * cos((i+1) * 2 * Pi/level));
			float y4 = (float)(_radiusBottom * sin((i+1) * 2 * Pi/level));

			Vector3D<float> p1(x1, y1, z);
			Vector3D<float> p2(x2, y2, z);
			Vector3D<float> p3(x3, y3, -z);
			Vector3D<float> p4(x4, y4, -z);

			(*mesh)[i*4+0] = Triangle<float>(p1, p3, p4);
			(*mesh)[i*4+1] = Triangle<float>(p1, p4, p2);

			//Construct triangles for the end-plates
			Vector3D<float> p5(0, 0,  z);
			Vector3D<float> p6(0, 0, -z);
			(*mesh)[i*4+2] = Triangle<float>(p1, p2, p5);
			(*mesh)[i*4+3] = Triangle<float>(p3, p6, p4);
		}
		return ownedPtr(mesh);
	}
}

bool Cone::doIsInside(const rw::math::Vector3D<>& point)
{
    // first test if the point z is within cone
    if(point[2]<-_height/2 || point[2]>_height/2)
        return false;
    // next calculate distance to z-axis (center of cone) and compare to distance
    // from cone surface to z-axis
    double distToZaxis = Vector2D<>(point[0],point[1]).norm2();
    double maxdistToZaxis;
    if( _radiusTop<_radiusBottom ){
        maxdistToZaxis = _radiusTop + (_height - (point[2]+_height/2)) * (_radiusBottom-_radiusTop);
    } else {
        maxdistToZaxis = _radiusBottom + (_height - (point[2]+_height/2)) * (_radiusTop-_radiusBottom);
    }
    return distToZaxis<maxdistToZaxis;
}

