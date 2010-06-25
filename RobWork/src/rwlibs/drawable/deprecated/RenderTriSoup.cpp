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


#include "RenderTriSoup.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

using namespace rw::common;

#define LINE_MAX_LENGTH 100

using namespace rwlibs::drawable;
using namespace rw::geometry;
using namespace rw::math;
namespace {

	void renderSoup(){

	}

}


RenderTriSoup::RenderTriSoup(const std::string &filename)
{
    loadTriFile(filename);
    _displayListId = glGenLists(1);
    rerender();
}

RenderTriSoup::RenderTriSoup()
{
	_displayListId = glGenLists(1);
	rerender();
}

RenderTriSoup::~RenderTriSoup()
{
	glDeleteLists(_displayListId, 1);
}

void RenderTriSoup::addFaces(
     const rw::geometry::TriMesh& faces,
    const double dr,
    const double dg,
    const double db)
{
    const float r = (float)dr;
    const float g = (float)dg;
    const float b = (float)db;

    int np = 0;

    _rgbArray.push_back(Rgb(r,g,b));
    _rgbToVertexMap.push_back(np);

    for(size_t i=0;i<faces.size();i++){
    	TriangleN0<double> tri = faces.getTriangle(i);
    	Vector3D<float> n = cast<float>( tri.calcFaceNormal() );

        Vector3D<float> v1 = cast<float>(tri[0]);
        _normalArray.push_back(n);
        _vertexArray.push_back(v1);
        np++;

        Vector3D<float> v2 = cast<float>(tri[1]);
        _normalArray.push_back(n);
        _vertexArray.push_back(v2);
        np++;

        Vector3D<float> v3 = cast<float>(tri[2]);
        _normalArray.push_back(n);
        _vertexArray.push_back(v3);
        np++;
    }
    _rgbToVertexMap.push_back(np);

    rerender();
}

void RenderTriSoup::clearFaces()
{
    std::cout << "Clear Faces\n";
    _vertexArray.clear();
    _normalArray.clear();
    _rgbArray.clear();
    _rgbToVertexMap.clear();

    rerender();
}

void RenderTriSoup::draw(DrawType type, double alpha) const
{
    switch (type) {
    case Render::SOLID:
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glCallList(_displayListId);
    	break;
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT, GL_FILL);
    	glCallList(_displayListId);
    case Render::WIRE:
    	glPolygonMode(GL_FRONT, GL_LINE);
    	glCallList(_displayListId);
    }
}

void RenderTriSoup::rerender()
{
    glNewList(_displayListId, GL_COMPILE);

    glPushMatrix();
    glBegin(GL_TRIANGLES);
    int curr_index=0;

    if (_rgbToVertexMap.size() > 0) {
        for(size_t i = 0; i<_rgbToVertexMap.size()-1; i++){
            glColor3f(_rgbArray[i].val[0],_rgbArray[i].val[1],_rgbArray[i].val[2]);
            for(;curr_index<_rgbToVertexMap[i+1];curr_index++){
                glNormal3fv(&_normalArray[curr_index][0]);
                glVertex3fv(&_vertexArray[curr_index][0]);
            }
        }
    }

    glEnd();
    glPopMatrix();

    glEndList();
}

void RenderTriSoup::loadTriFile(const std::string &filename)
{
    std::ifstream input_stream(filename.c_str());
    if (!input_stream.is_open()) {
        RW_THROW("Can't open file " << StringUtil::quote(filename));
    }

    char *next;
    char  token[LINE_MAX_LENGTH];
    int   width;
    char input[LINE_MAX_LENGTH];
    int nb_points = 0;

    // while characters still exists and no errors occour
    while (  input_stream.fail()==0 && input_stream.eof()==0 ){
        //  Read the next line of the file into INPUT.
        input_stream.getline(input, LINE_MAX_LENGTH);
        //  Advance to the first nonspace character in INPUT.
        for ( next = input; *next != '\0' && *next == 32; next++ ){}
        // 32==SPACE character

        //  Skip blank lines and comments and linebreaks.
        if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$'  ){
            continue;
        }
        //  Extract the first word in this line.
        sscanf ( next, "%s%n", token, &width );

        //  Set NEXT to point to just after this token.
        next = next + width;

        if ( !strcmp( token, "color" ) ){
            float r,g,b;
            sscanf ( next, "%e %e %e", &r, &g, &b );
            Rgb rgb(r,g,b);
            _rgbArray.push_back(rgb);
            _rgbToVertexMap.push_back(nb_points);
        } else if( !strcmp( token, "point" ) ){
            float x,y,z,nx,ny,nz;
            sscanf ( next, "%e %e %e %e %e %e", &x, &y, &z, &nx, &ny, &nz );
            Vector3D<float> n(nx,ny,nz);
            Vector3D<float> v(x,y,z);
            _normalArray.push_back(n);
            _vertexArray.push_back(v);
            nb_points++;
        } else {
            RW_THROW("unrecognized keyword " << StringUtil::quote(token));
        }
    }

    _rgbToVertexMap.push_back(nb_points);
}
