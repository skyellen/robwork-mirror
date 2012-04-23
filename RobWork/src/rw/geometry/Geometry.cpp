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


#include "Geometry.hpp"

#include <rw/math/Math.hpp>

#include "Sphere.hpp"
#include "Box.hpp"
#include "Cylinder.hpp"
#include "Cone.hpp"

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

namespace {

	std::string makeName(GeometryData::GeometryType gtype){
		int ri = Math::ranI(0xFF,0xFFFFFF);
		std::stringstream sstr;
		sstr << GeometryData::toString(gtype) << "_" << ri;
		return sstr.str();
	}

}

Geometry::Geometry(GeometryData::Ptr data, double scale):
    _refFrame(NULL),
	_data(data),
	_transform(rw::math::Transform3D<>::identity() ),
	_scale(scale),
	_name(makeName(data->getType())),
	_mask(Geometry::CollisionGroup)
{

};

Geometry::Geometry(GeometryData::Ptr data,
		 const rw::math::Transform3D<>& t3d,
		 double scale):
	_refFrame(NULL),
	_data(data),
	_transform(t3d),
	_scale(scale),
	_name(makeName(data->getType())),
	_mask(Geometry::CollisionGroup)
{
};

Geometry::~Geometry(){};

Geometry::Ptr Geometry::makeSphere(double radi){
    return ownedPtr(new Geometry(ownedPtr(new Sphere(radi))));
}

Geometry::Ptr Geometry::makeBox(double x, double y, double z){
    return ownedPtr(new Geometry(ownedPtr(new Box(x,y,z))));
}

Geometry::Ptr Geometry::makeCone(double height, double radiusTop, double radiusBot){
    return ownedPtr(new Geometry(ownedPtr(new Cone(height,radiusTop,radiusBot))));
}

Geometry::Ptr Geometry::makeCylinder(float radius, float height){
    return ownedPtr(new Geometry(ownedPtr(new Cylinder(radius, height))));
}
