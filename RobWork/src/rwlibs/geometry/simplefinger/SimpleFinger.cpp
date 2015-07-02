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

#include "SimpleFinger.hpp"

#include <rwlibs/csg/CSGModel.hpp>
#include <rwlibs/csg/CSGModelFactory.hpp>



using namespace rwlibs::geometry::simplefinger;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::csg;



SimpleFinger::SimpleFinger() :
	_length(0.1),
	_width(0.025),
	_depth(0.01),
	_chflength(0.0),
	_chfdepth(0.0),
	_cutpos(0.075),
	_cutdepth(0.0),
	_cutangle(120.0),
	_cuttilt(0.0)
{
}


SimpleFinger::~SimpleFinger() {
}


SimpleFinger::SimpleFinger(const rw::math::Q& initQ) {
	setParameters(initQ);
}


TriMesh::Ptr SimpleFinger::createMesh(int resolution) const {
	/* make base geometry */
	CSGModel::Ptr base = CSGModelFactory::makeBox(_length, _width, _depth);
	base->translate(_length/2, 0, _depth/2);
	
	/* make chamfering */
	Vector3D<> point(_length - _chflength, 0.0, _depth);
	Vector3D<> normal = Vector3D<>(_chfdepth, 0.0, _chflength);
	base->subtract(CSGModelFactory::makePlane(point, -normal));
	
	/* make cutout */
	CSGModel::Ptr cutout = CSGModelFactory::makeWedge(_cutangle*Deg2Rad);
	cutout->rotate(-90*Deg2Rad, 90*Deg2Rad, 0);
	cutout->rotate(_cuttilt*Deg2Rad, 0, 0);
	cutout->translate(_cutpos, 0, _cutdepth);
	base->subtract(cutout);
	
	TriMesh::Ptr mesh = base->getTriMesh();

	return mesh;
}


Q SimpleFinger::getParameters() const {
	Q q(9);
	
	q[0] = _length;
	q[1] = _width;
	q[2] = _depth;
	q[3] = _chflength;
	q[4] = _chfdepth;
	q[5] = _cutpos;
	q[6] = _cutdepth;
	q[7] = _cutangle;
	q[8] = _cuttilt;
	
	return q;
}

void SimpleFinger::setParameters(const rw::math::Q& q) {
	if (q.size() != 9) {
		RW_THROW("Size of parameter list must be 9!");
	}
	
	_length = q[0];
	_width = q[1];
	_depth = q[2];
	_chflength = q[3];
	_chfdepth = q[4];
	_cutpos = q[5];
	_cutdepth = q[6];
	_cutangle = q[7];
	_cuttilt = q[8];
}
