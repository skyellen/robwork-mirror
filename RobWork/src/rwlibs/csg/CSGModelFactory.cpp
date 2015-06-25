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

#include "CSGModelFactory.hpp"

#include <rw/geometry/Box.hpp>
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Sphere.hpp>



using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::csg;



CSGModel::Ptr CSGModelFactory::makeBox(float x, float y, float z)
{
	Box box(x, y, z);
	
	return ownedPtr(new CSGModel(*(box.createMesh(0))));
}

CSGModel::Ptr CSGModelFactory::makeCylinder(float r, float h)
{
	Cylinder cyl(r, h);
	
	return ownedPtr(new CSGModel(*(cyl.createMesh(32))));
}

CSGModel::Ptr CSGModelFactory:: makeSphere(float r)
{
	Sphere sph(r, 4);
	
	return ownedPtr(new CSGModel(*(sph.createMesh(3))));
}

CSGModel::Ptr CSGModelFactory::makePlane(Vector3D<> point, Vector3D<> normal)
{
	const float planeSize = 100.0;
	
	normal = normalize(normal);
	Transform3D<> t = Transform3D<>::makeLookAt(point, point+normal, Vector3D<>::y());
	
	CSGModel::Ptr csgmodel = makeBox(planeSize, planeSize, planeSize);
	
	csgmodel->translate(0, 0, planeSize/2);
	csgmodel->transform(t);
	
	return csgmodel;
}

CSGModel::Ptr CSGModelFactory::makeWedge(float angle)
{
	CSGModel::Ptr wedge = makePlane(Vector3D<>(), Vector3D<>(-sin(angle/2), -cos(angle/2), 0));
	(*wedge) *= (*makePlane(Vector3D<>(), Vector3D<>(-sin(angle/2), cos(angle/2), 0)));
	
	return wedge;
}
