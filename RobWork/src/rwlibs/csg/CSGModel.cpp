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

#include "CSGModel.hpp"

#include "CSGConvert.hpp"



using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::csg;



CSGModel::CSGModel() :
	_needsConversion(false)
{}


CSGModel::CSGModel(const CSGModel& csgmodel) :
	_needsConversion(csgmodel._needsConversion),
	_model(csgmodel._model)
{
	if (csgmodel._mesh != NULL) {
		_mesh = csgmodel._mesh->clone();
	}
}


CSGModel::CSGModel(const TriMesh& trimesh) :
	_needsConversion(false),
	_mesh(trimesh.clone())
{
	_model = CSGConvert::TriMesh2csgjs_model(trimesh);
}


void CSGModel::translate(float x, float y, float z)
{
	Transform3D<> t(Vector3D<>(x, y, z), Rotation3D<>());
	
	transform(t);
}


void CSGModel::rotate(float r, float p, float y)
{
	Transform3D<> t(Vector3D<>(), RPY<>(r, p, y).toRotation3D());
	
	transform(t);
}


void CSGModel::transform(const math::Transform3D<>& T)
{
	// convert all of the model vertices to Vector3D & apply transform
	for (unsigned i = 0; i < _model->vertices.size(); ++i) {
		Vector3D<> v(_model->vertices[i].pos.x, _model->vertices[i].pos.y, _model->vertices[i].pos.z);
		
		v = T * v;
		
		_model->vertices[i].pos.x = v[0];
		_model->vertices[i].pos.y = v[1];
		_model->vertices[i].pos.z = v[2];
	}
	
	_needsConversion = true;
}


void CSGModel::add(CSGModel::Ptr model) {
	_model = ownedPtr(new csgjs_model(csgjs_union(*_model, *model->_model)));
	
	_needsConversion = true;
}


void CSGModel::subtract(CSGModel::Ptr model)
{
	_model = ownedPtr(new csgjs_model(csgjs_difference(*_model, *model->_model)));
	
	_needsConversion = true;
}


void CSGModel::intersect(CSGModel::Ptr model)
{
	_model = ownedPtr(new csgjs_model(csgjs_intersection(*_model, *model->_model)));
	
	_needsConversion = true;
}


TriMesh::Ptr CSGModel::getTriMesh()
{
	// if the model was changed, apply conversion
	if (_needsConversion) _convertToTriMesh();
	
	return _mesh;
}

void CSGModel::_convertToTriMesh()
{
	_mesh = CSGConvert::csgjs_model2TriMesh(_model);
}
