/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Face.hpp"
#include "Curve.hpp"
#include "Surface.hpp"

#include <list>

using namespace rw::geometry;
using namespace rw::math;

Face::Face():
	_resolution(10)
{
}

Face::~Face() {
}

rw::common::Ptr<TriMesh> Face::getTriMesh(bool) const {
	// Construct loop polygon
	std::vector<Vector3D<> > polygon3d;
	for (std::size_t i = 0; i < curveCount(); i++) {
		const Curve& c = getCurve(i);
		const std::list<Vector3D<> > points = c.discretizeAdaptive(_resolution);
		polygon3d.insert(polygon3d.end(),points.begin(),--points.end());
	}
	return surface().getTriMesh(polygon3d);
}

std::pair<double,double> Face::extremums(const Vector3D<>& dir) const {
	std::pair<double,double> res(std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	double& min = res.first;
	double& max = res.second;

	// Check edges
	for (std::size_t i = 0; i < curveCount(); i++) {
		const Curve& c = getCurve(i);
		const std::pair<double,double> extremums = c.extremums(dir);
		if (extremums.first < min)
			min = extremums.first;
		if (extremums.second > max)
			max = extremums.second;
	}

	// Deal with surface itself
	const std::pair<double,double> extremums = surface().extremums(dir);
	if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min)
		min = extremums.first;
	if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max)
		max = extremums.second;

	return res;
}

OBB<> Face::obb() {
	const rw::common::Ptr<const TriMesh> mesh = getTriMesh();
	const OBB<> obb = OBB<>::buildTightOBB(*mesh);

	const Rotation3D<> R = obb.getTransform().R();
	const Vector3D<> e1 = R.getCol(0);
	const Vector3D<> e2 = R.getCol(1);
	const Vector3D<> e3 = R.getCol(2);

	std::vector<double> min(3,std::numeric_limits<double>::max());
	std::vector<double> max(3,-std::numeric_limits<double>::max());

	// Handle edges
	for (std::size_t ci = 0; ci < curveCount(); ci++) {
		const Curve& c = getCurve(ci);
		for (std::size_t i = 0; i < 3; i++) {
			const std::pair<double,double> extremums = c.extremums(R.getCol(i));
			if (extremums.first < min[i])
				min[i] = extremums.first;
			if (extremums.second > max[i])
				max[i] = extremums.second;
		}
	}

	// Deal with surface
	for (std::size_t i = 0; i < 3; i++) {
		const std::pair<double,double> extremums = surface().extremums(R.getCol(i));
		if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min[i])
			min[i] = extremums.first;
		if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max[i])
			max[i] = extremums.second;
	}

	const Vector3D<> P = Vector3D<>((max[0]+min[0])*e1+(max[1]+min[1])*e2+(max[2]+min[2])*e3)/2;
	const Vector3D<> halfLen = Vector3D<>(max[0]-min[0],max[1]-min[1],max[2]-min[2])/2;
	return OBB<>(Transform3D<>(P,R), halfLen);
}
