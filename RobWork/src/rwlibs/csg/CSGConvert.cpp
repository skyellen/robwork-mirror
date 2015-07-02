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
 
#include "CSGConvert.hpp"

#include <rw/rw.hpp>
#include <rw/geometry/TriMesh.hpp>

#define CSGJS_HEADER_ONLY
#include <csgjs/csgjs.cpp>


using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::csg;



Ptr<csgjs_model> CSGConvert::TriMesh2csgjs_model(const geometry::TriMesh& mesh)
{
	csgjs_model* model = new csgjs_model;
	
	// iterate over all triangles
	for (unsigned i = 0; i < mesh.getSize(); ++i) {
		const Triangle<double>& t = mesh.getTriangle(i);
		
		
		// get triangle normal
		Vector3D<double> tnormal = t.calcFaceNormal();
		
		// create vertices
		Vector3D<double> tv;
		csgjs_vertex v;
		v.normal = csgjs_vector(tnormal[0], tnormal[1], tnormal[2]);
		
		tv = t[0];
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+0);
		
		tv = t[1];
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+1);
		
		tv = t[2];
		v.pos = csgjs_vector(tv[0], tv[1], tv[2]);
		model->vertices.push_back(v);
		model->indices.push_back(3*i+2);
	}
	
	return ownedPtr(model);
}



TriMesh::Ptr CSGConvert::csgjs_model2TriMesh(rw::common::Ptr<csgjs_model> model)
{
	PlainTriMeshD* mesh = new PlainTriMeshD;
	
	// loop over all triangles
	for (unsigned i = 0; i < (model->indices.size() / 3); ++i) {
		csgjs_vector v1 = model->vertices[model->indices[3*i+0]].pos;
		csgjs_vector v2 = model->vertices[model->indices[3*i+1]].pos;
		csgjs_vector v3 = model->vertices[model->indices[3*i+2]].pos;
			
		Vector3D<double> p1(v1.x, v1.y, v1.z);
		Vector3D<double> p2(v2.x, v2.y, v2.z);
		Vector3D<double> p3(v3.x, v3.y, v3.z);
		
		mesh->add(Triangle<double>(p1, p2, p3));
	}
	
	return ownedPtr(mesh);
}
