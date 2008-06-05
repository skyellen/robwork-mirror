/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "RenderTriSoup.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/geometry/GeometrySTL.hpp>
using namespace rw::common;

#define LINE_MAX_LENGTH 100

using namespace rwlibs::drawable;
using namespace rw::geometry;

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
    const std::vector<Face<float> >& faces,
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

    typedef std::vector<Face<float> >::const_iterator I;
    for (I it = faces.begin(); it != faces.end(); ++it) {
        Normal n((*it)._normal[0], (*it)._normal[1], (*it)._normal[2]);
        Vertex v1((*it)._vertex1[0], (*it)._vertex1[1], (*it)._vertex1[2]);
        _normalArray.push_back(n);
        _vertexArray.push_back(v1);
        np++;

        Vertex v2((*it)._vertex2[0], (*it)._vertex2[1], (*it)._vertex2[2]);
        _normalArray.push_back(n);
        _vertexArray.push_back(v2);
        np++;

        Vertex v3((*it)._vertex3[0], (*it)._vertex3[1], (*it)._vertex3[2]);
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
                glNormal3fv(_normalArray[curr_index].val);
                glVertex3fv(_vertexArray[curr_index].val);
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
            Normal n(nx,ny,nz);
            Vertex v(x,y,z);
            _normalArray.push_back(n);
            _vertexArray.push_back(v);
            nb_points++;
        } else {
            RW_THROW("unrecognized keyword " << StringUtil::quote(token));
        }
    }

    _rgbToVertexMap.push_back(nb_points);
}
