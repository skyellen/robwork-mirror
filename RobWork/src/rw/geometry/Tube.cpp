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

Tube::Tube(float radius, float thickness, float height):
	_radius(radius),_thickness(thickness),_height(height)
{
}

Tube::Tube(const rw::math::Q& initQ)
{
	setParameters(initQ);
}

Tube::~Tube()
{
}

float Tube::getInnerRadius() const {
	return _radius;
}

float Tube::getThickness() const {
	return _thickness;
}

float Tube::getHeight() const {
	return _height;
}

TriMesh::Ptr Tube::createMesh(int level) const{
	if(level < 0)
		level = 16; // default

	PlainTriMeshF *mesh = new PlainTriMeshF(8*level);

	float z = _height/2.0f;

	for (int i = 0; i < level; i++) {
		float x1in = static_cast<float>((_radius) * cos(i * 2 * Pi/level));
		float y1in = static_cast<float>((_radius) * sin(i * 2 * Pi/level));

		float x2in = static_cast<float>((_radius) * cos((i+1) * 2 * Pi/level));
		float y2in = static_cast<float>((_radius) * sin((i+1) * 2 * Pi/level));

		float x1out = static_cast<float>((_radius+_thickness) * cos(i * 2 * Pi/level));
		float y1out = static_cast<float>((_radius+_thickness) * sin(i * 2 * Pi/level));

		float x2out = static_cast<float>((_radius+_thickness) * cos((i+1) * 2 * Pi/level));
		float y2out = static_cast<float>((_radius+_thickness) * sin((i+1) * 2 * Pi/level));

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
	Q q(3);
	q(0) = _radius;
	q(1) = _thickness;
	q(2) = _height;
	return q;
}

void Tube::setParameters(const rw::math::Q& q) {
	if (q.size() != 3) {
		RW_THROW("Size of parameter list must equal to 3!");
	}
	
	_radius = static_cast<float>(q(0));
	_thickness = static_cast<float>(q(1));
	_height = static_cast<float>(q(2));
}

bool Tube::doIsInside(const Vector3D<>& point) {
	const double distXY = Vector2D<>(point[0],point[1]).norm2();
    return fabs(point[2])<=_height/2. && distXY >= _radius && distXY <= _radius+_thickness;
}
