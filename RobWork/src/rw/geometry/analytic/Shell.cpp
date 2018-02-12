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

#include "Shell.hpp"
#include "Face.hpp"
#include "GenericFace.hpp"

#include <rw/geometry/PlainTriMesh.hpp>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

Shell::Shell() {
}

Shell::~Shell() {
}

TriMesh::Ptr Shell::getTriMesh(bool forceCopy) {
	const PlainTriMeshN1D::Ptr mesh = ownedPtr(new PlainTriMeshN1D());
	const std::size_t N = size();
	GenericFace face;
	for(std::size_t i = 0; i < N; i++) {
		getFace(i, face);
		const PlainTriMeshN1D::Ptr faceMesh = face.getTriMesh().scast<PlainTriMeshN1D>();
		mesh->add(faceMesh);
	}
	return mesh;
}

TriMesh::Ptr Shell::getTriMesh(bool forceCopy) const {
	const PlainTriMeshN1D::Ptr mesh = ownedPtr(new PlainTriMeshN1D());
	const std::size_t N = size();
	GenericFace face;
	for(std::size_t i = 0; i < N; i++) {
		getFace(i, face);
		const PlainTriMeshN1D::Ptr faceMesh = face.getTriMesh().scast<PlainTriMeshN1D>();
		mesh->add(faceMesh);
	}
	return mesh;
}

std::pair<double,double> Shell::extremums(const Vector3D<>& dir) const {
	std::pair<double,double> res(std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	double& min = res.first;
	double& max = res.second;

	const std::size_t N = size();
	GenericFace face;
	for(std::size_t i = 0; i < N; i++) {
		getFace(i, face);
		const std::pair<double,double> extremums = face.extremums(dir);
		if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min)
			min = extremums.first;
		if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max)
			max = extremums.second;
	}

	return res;
}

OBB<> Shell::obb() const {
	const TriMesh::Ptr mesh = getTriMesh();
	const OBB<> obb = OBB<>::buildTightOBB(*mesh);

	const Rotation3D<> R = obb.getTransform().R();
	const Vector3D<> e1 = R.getCol(0);
	const Vector3D<> e2 = R.getCol(1);
	const Vector3D<> e3 = R.getCol(2);

	std::vector<double> min(3,std::numeric_limits<double>::max());
	std::vector<double> max(3,-std::numeric_limits<double>::max());

	const std::size_t N = size();
	GenericFace face;
	for(std::size_t fi = 0; fi < N; fi++) {
		getFace(fi, face);
		for(std::size_t i = 0; i < 3; i++) {
			const std::pair<double,double> extremums = face.extremums(R.getCol(i));
			if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min[i])
				min[i] = extremums.first;
			if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max[i])
				max[i] = extremums.second;
		}
	}

	const Vector3D<> P = Vector3D<>((max[0]+min[0])*e1+(max[1]+min[1])*e2+(max[2]+min[2])*e3)/2;
	const Vector3D<> halfLen = Vector3D<>(max[0]-min[0],max[1]-min[1],max[2]-min[2])/2;
	return OBB<>(Transform3D<>(P,R), halfLen);
}

void Shell::getFace(std::size_t idx, GenericFace& face) const {
	face = *getFace(idx);
}
