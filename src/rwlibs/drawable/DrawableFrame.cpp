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
#include "DrawableFrame.hpp"

#include <iostream>

using namespace rwlibs::drawable;

DrawableFrame::DrawableFrame(float size):
    _size(size),
    _displaylist(0)
{
    _quadratic=gluNewQuadric();
    update(CUSTOM);
}

void DrawableFrame::draw() const
{
    glCallList(_displaylist);
}

void DrawableFrame::setHighlighted(bool b){}

void DrawableFrame::update(UpdateType type)
{
    if (_displaylist != 0) //Delete the old displaylist if such exists
        glDeleteLists(_displaylist, 1);
    _displaylist = glGenLists(1);

    glNewList(_displaylist, GL_COMPILE);
    glPushMatrix();
    const float REL_WIDTH = 0.05f;
    float width = 0.5;

    if (_highlighted) width = 2;

    switch (_drawType) {
    case SOLID:
    case OUTLINE: // Draw nice frame
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // Nice frame
        // Draw z-axis
        glColor4f(0.0f, 0.0f, 1.0f, _alpha); // Blue color
        gluCylinder(
            _quadratic,
            width*_size*REL_WIDTH,
            width*_size*REL_WIDTH,
            _size,
            32,
            32);    // Draw Our Cylinder

        glTranslatef(0.0f,0.0f,_size);// Center The Cone

        gluCylinder(
            _quadratic,_size*2*REL_WIDTH,0.0f,_size*2*REL_WIDTH,32,32); // A Cone

        // Draw x-axis
        glColor4f(1.0f, 0.0f, 0.0f, _alpha); // Red color
        glTranslatef(0.0f,0.0f,-_size); // Center The Cylinder
        glRotatef(90.0f,0.0f,1.0f,0.0f); // Rotate around y-ax
        gluCylinder(
            _quadratic,
            width*_size*REL_WIDTH,
            width*_size*REL_WIDTH,
            _size,
            32,
            32);    // Draw Our Cylinder

        glTranslatef(0.0f,0.0f,_size);// Center The Cone
        gluCylinder(
            _quadratic,_size*2*REL_WIDTH,0.0f,_size*2*REL_WIDTH,32,32);// A Cone

        // Draw y-axis
        glColor4f(0.0f, 1.0f, 0.0f, _alpha); // Green color
        glTranslatef(0.0f,0.0f,-_size);                     // Center The Cylinder
        glRotatef(90.0f,-1.0f,0.0f,0.0f); // Rotate around y-axis
        gluCylinder(
            _quadratic,
            width*_size*REL_WIDTH,
            width*_size*REL_WIDTH,
            _size,
            32,
            32);    // Draw Our Cylinder

        glTranslatef(0.0f,0.0f,_size);// Center The Cone
        gluCylinder(
            _quadratic,
            _size*2*REL_WIDTH,
            0.0f,
            _size*3*REL_WIDTH,
            32,
            32);// A Cone

        break;
    case WIRE: // Draw simple frame
        glLineWidth(width);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0,0,0);
        glVertex3d(_size,0,0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0,0,0);
        glVertex3d(0,_size,0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0,0,0);
        glVertex3d(0,0,_size);
        glEnd();
        break;
    }

    glPopMatrix();
    glEndList();
}
