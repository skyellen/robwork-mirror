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


#include "RenderSTL.hpp"

#include <rw/geometry/Geometry.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/STLFile.hpp>

using namespace rwlibs::drawable;
using namespace rw::geometry;

using namespace rw::geometry;
using namespace rwlibs::drawable;

namespace
{

    void setArray4(float *array, float v0, float v1, float v2, float v3 ){
    	array[0]=v0;
    	array[1]=v1;
    	array[2]=v2;
    	array[3]=v3;
    }
}

RenderSTL::RenderSTL(const std::string &filename):
	_r(0.8f),_g(0.8f),_b(0.8f)
{
	PlainTriMesh<TriangleN1<float> >* faces = STLFile::read(filename);
	_faces = faces;

	//_renderer = new RenderGeometry( _id, new FaceArrayGeometry(_faces));
	setArray4(_diffuse, 0.8f,0.8f,0.8f,1.0f);
	setArray4(_ambient, 0.2f,0.2f,0.2f,1.0f);
	setArray4(_emission, 0.0f,0.0f,0.0f,0.0f);
	setArray4(_specular, 0.2f,0.2f,0.2f,1.0f);
	_shininess[0] = 128;

	// create displaylist
    _displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPushMatrix();
    glBegin(GL_TRIANGLES);
    // Draw all faces.
    // TODO: faces should have norma

    for(size_t i=0;i<faces->size();i++){
    	TriangleN1<float> &tri = (*faces)[i];
        glNormal3fv(&tri.getFaceNormal()[0]);
        glVertex3fv(&tri[0][0]);
        glVertex3fv(&tri[1][0]);
        glVertex3fv(&tri[2][0]);
    }

    glEnd();
    glPopMatrix();
    glEndList();
    delete faces;
}

void RenderSTL::setFaces(rw::geometry::TriMeshPtr faces) {
    _faces = faces;
}


const rw::geometry::TriMeshPtr RenderSTL::getFaces() const {
    return _faces;
}

void RenderSTL::draw(DrawType type, double alpha) const{
	glColor4f(_r, _g, _b, (float)alpha);
	_diffuse[3] = (float)alpha;
	glMaterialfv(GL_FRONT, GL_AMBIENT, _ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, _diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, _specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, _shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, _emission);
	//glPushAttrib(GL_CURRENT_BIT);

	switch(type){
    case Render::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
		glCallList(_displayListId);
		break;
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
		glCallList(_displayListId);
    case Render::WIRE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_LINE);
    	glCallList(_displayListId);
    	break;
	}
}
