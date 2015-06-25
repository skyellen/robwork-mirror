/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Tube.hpp"

#include <rw/math/Constants.hpp>
#include <rw/common/macros.hpp>
#include "PlainTriMesh.hpp"
#include <rw/math/Vector2D.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

Tube::Tube(float radius, float height):
	_radius(radius),_height(height)
{
}

Tube::Tube(const rw::math::Q& initQ)
{
	setParameters(initQ);
}

Tube::~Tube()
{
}

float Tube::getRadius() const {
	return _radius;
}

float Tube::getHeight() const {
	return _height;
}

TriMesh::Ptr Tube::createMesh(int resolution) const{
	static double THICKNESS = 0.00001; // plus/minus 0.01 mm for inside/outside surface

	int level = resolution;
	if(resolution<0)
		level = 16; // default

	PlainTriMeshF *mesh = new PlainTriMeshF(8*level);

	float z = _height/2.0f;

	for (int i = 0; i < level; i++) {
		float x1in = (float)((_radius-THICKNESS) * cos(i * 2 * Pi/level));
		float y1in = (float)((_radius-THICKNESS) * sin(i * 2 * Pi/level));

		float x2in = (float)((_radius-THICKNESS) * cos((i+1) * 2 * Pi/level));
		float y2in = (float)((_radius-THICKNESS) * sin((i+1) * 2 * Pi/level));

		float x1out = (float)((_radius+THICKNESS) * cos(i * 2 * Pi/level));
		float y1out = (float)((_radius+THICKNESS) * sin(i * 2 * Pi/level));

		float x2out = (float)((_radius+THICKNESS) * cos((i+1) * 2 * Pi/level));
		float y2out = (float)((_radius+THICKNESS) * sin((i+1) * 2 * Pi/level));

		Vector3D<float> p1in(x1in, y1in, z);
		Vector3D<float> p2in(x1in, y1in, -z);
		Vector3D<float> p3in(x2in, y2in, z);
		Vector3D<float> p4in(x2in, y2in, -z);

		Vector3D<float> p1out(x1out, y1out, z);
		Vector3D<float> p2out(x1out, y1out, -z);
		Vector3D<float> p3out(x2out, y2out, z);
		Vector3D<float> p4out(x2out, y2out, -z);

		// Inner
		(*mesh)[i*8+0] = Triangle<float>(p3in,p2in,p1in);
		(*mesh)[i*8+1] = Triangle<float>(p3in,p4in,p2in);

		// Outer
		(*mesh)[i*8+2] = Triangle<float>(p1out,p2out,p3out);
		(*mesh)[i*8+3] = Triangle<float>(p2out,p4out,p3out);

		// Top
		(*mesh)[i*8+4] = Triangle<float>(p3out,p3in,p1in);
		(*mesh)[i*8+5] = Triangle<float>(p1out,p3out,p1in);

		// Bottom
		(*mesh)[i*8+6] = Triangle<float>(p2in,p4in,p4out);
		(*mesh)[i*8+7] = Triangle<float>(p2in,p4out,p2out);
	}
	return ownedPtr(mesh);
}

rw::math::Q Tube::getParameters() const {
	Q q(2);
	q(0) = _height;
	q(1) = _radius;
	return q;
}

void Tube::setParameters(const rw::math::Q& q) {
	if (q.size() != 2) {
		RW_THROW("Size of parameter list must equal 2!");
	}
	
	_height = q(0);
	_radius = q(1);
}

bool Tube::doIsInside(const rw::math::Vector3D<>& point){
    return point[2]<_height && rw::math::Vector2D<>(point[0],point[1]).norm2()<_radius;
}
