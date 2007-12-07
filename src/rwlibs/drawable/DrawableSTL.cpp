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

#include "DrawableSTL.hpp"

#include <rw/geometry/GeometrySTL.hpp>
#include <rw/math/Vector3D.hpp>

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::geometry;

DrawableSTL::DrawableSTL(const std::string &filename, float r, float g, float b)
    : _r(r), _g(g), _b(b)
{
    GeometrySTL::ReadSTL(filename, _vfaces);
    update(CUSTOM);
}

namespace
{
    void drawFace(const Face<float>& face)
    {
        glNormal3fv(face._normal);
        glVertex3fv(face._vertex1);
        glVertex3fv(face._vertex2);
        glVertex3fv(face._vertex3);
    }
}


void DrawableSTL::setFaces(const std::vector<Face<float> >& faces) {
    _vfaces = faces;
}


void DrawableSTL::update(UpdateType type)
{
    if (type == CUSTOM || type == ALPHA) {
        if (_displayListId != 0) {
            glDeleteLists(_displayListId, 1);
        }

        _displayListId = glGenLists(1);
        glNewList(_displayListId, GL_COMPILE);
        glPushMatrix();
        glColor4f(_r, _g, _b, _alpha);
        glBegin(GL_TRIANGLES);

        // Draw all faces.
        std::for_each(_vfaces.begin(), _vfaces.end(), drawFace);

        glEnd();
        glPopMatrix();
        glEndList();
    }
}

void DrawableSTL::setColor(float r, float g, float b)
{
    _r = r;
    _g = g;
    _b = b;
    update(CUSTOM);
}


