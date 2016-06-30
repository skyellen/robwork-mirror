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


#include "RenderGeometry.hpp"
#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Vector3D.hpp>
#include "DrawableUtil.hpp"

using namespace rwlibs::opengl;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::math;

namespace
{
    void setArray4(float *array, float v0, float v1, float v2, float v3 ){
    	array[0]=v0;
    	array[1]=v1;
    	array[2]=v2;
    	array[3]=v3;
    }
}


RenderGeometry::RenderGeometry(Geometry::Ptr geometry):
		_geometry(geometry), _r(0.8f),_g(0.8f),_b(0.8f)
{
    setGeometry(_geometry);
}

RenderGeometry::RenderGeometry(rw::geometry::TriMesh::Ptr mesh):
        _geometry(rw::common::ownedPtr( new Geometry(mesh) ) ), _r(0.8f),_g(0.8f),_b(0.8f)
{
    setGeometry(_geometry);
}

void RenderGeometry::setGeometry(rw::geometry::Geometry::Ptr geom){
    _geometry = geom;
    GeometryData::Ptr geomdata = geom->getGeometryData();
    _mesh = geomdata->getTriMesh(false);
}

RenderGeometry::~RenderGeometry() {
}

void RenderGeometry::setColor(float r, float g, float b) {
    _r = r;
    _g = g;
    _b = b;
}

void RenderGeometry::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{
    if(_geometry==NULL)
        return;

    glPushMatrix();

	float scale = (float)_geometry->getScale();
	if (scale != 1.0)
		glScalef(scale, scale, scale);

	DrawableUtil::multGLTransform(_geometry->getTransform());

	glColor4f(_r, _g, _b, (float)alpha);
	switch(type){
    case DrawableNode::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	render();
		break;
    case DrawableNode::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
    	render();
    case DrawableNode::WIRE: // Draw nice frame
    	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    	render();
    	break;
	}
	glPopMatrix();
}

void RenderGeometry::render() const{
    //glPopAttrib(); // pop color and material attributes
    glBegin(GL_TRIANGLES);
    // Draw all faces.
    for(size_t i=0;i<_mesh->getSize();i++){
        Triangle<double> tri = _mesh->getTriangle(i);
        Vector3D<float> n = cast<float>(tri.calcFaceNormal());
        Vector3D<float> v0 = cast<float>(tri[0]);
        Vector3D<float> v1 = cast<float>(tri[1]);
        Vector3D<float> v2 = cast<float>(tri[2]);
        glNormal3fv(&n[0]);
        glVertex3fv(&v0[0]);
        glVertex3fv(&v1[0]);
        glVertex3fv(&v2[0]);
    }
    glEnd();
}


