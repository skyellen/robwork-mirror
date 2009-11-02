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

#include "RenderFrame.hpp"

#include <iostream>

using namespace rwlibs::drawable;

namespace {
	void renderWire(float width, float size){
        glLineWidth(width);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0,0,0);
        glVertex3d(size,0,0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0,0,0);
        glVertex3d(0,size,0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0,0,0);
        glVertex3d(0,0,size);
        glEnd();
	};

	void renderSolid(float alpha, float width, float size, GLUquadricObj *quad){
		const float REL_WIDTH = 0.05f;
        // Nice frame
        // Draw z-axis
        glColor4f(0.0f, 0.0f, 1.0f, alpha); // Blue color
		gluCylinder(quad, width*size*REL_WIDTH, width*size*REL_WIDTH, size, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,size);// Center The Cone
        gluCylinder(quad,size*2*REL_WIDTH,0.0f,size*2*REL_WIDTH,32,32); // A Cone

        // Draw x-axis
        glColor4f(1.0f, 0.0f, 0.0f, alpha); // Red color
        glTranslatef(0.0f,0.0f,-size); // Center The Cylinder
        glRotatef(90.0f,0.0f,1.0f,0.0f); // Rotate around y-ax
        gluCylinder(quad, width*size*REL_WIDTH, width*size*REL_WIDTH, size, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,size);// Center The Cone
        gluCylinder(quad,size*2*REL_WIDTH,0.0f,size*2*REL_WIDTH,32,32);// A Cone

        // Draw y-axis
        glColor4f(0.0f, 1.0f, 0.0f, alpha); // Green color
        glTranslatef(0.0f,0.0f,-size);                     // Center The Cylinder
        glRotatef(90.0f,-1.0f,0.0f,0.0f); // Rotate around y-axis
        gluCylinder(quad, width*size*REL_WIDTH, width*size*REL_WIDTH, size, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,size);// Center The Cone
        gluCylinder(quad, size*2*REL_WIDTH, 0.0f, size*3*REL_WIDTH, 32, 32);// A Cone
	};

	void initializeColors(float *r, float *g, float *b){
		r[0] = 1.0; r[1] = 0.0; r[2] = 0.0; r[3] = 1.0;
		g[0] = 0.0; g[1] = 1.0; g[2] = 0.0; g[3] = 1.0;
		b[0] = 0.0; b[1] = 0.0; b[2] = 1.0; b[3] = 1.0;
	};
}

RenderFrame::RenderFrame(float size):
    _size(size),
    _displayListId(0)
{
	initializeColors(_red,_green,_blue);
    _quadratic = gluNewQuadric();

    /*_displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPushMatrix();
    renderSolid(1.0, 0.3, _size, _quadratic);
    glPopMatrix();
    glEndList();*/

}

void RenderFrame::draw(DrawType type, double alpha) const
{
	const float width = 0.3;
	_green[3] = alpha;
	_red[3] = alpha;
	_blue[3] = alpha;
    switch (type) {
    case Render::SOLID:
    case Render::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    	/*glColor4fv(_green);
    	glPushAttrib(GL_CURRENT_COLOR);
    	glColor4fv(_red);
    	glPushAttrib(GL_CURRENT_COLOR);
    	glColor4fv(_blue);
    	//glPushAttrib(GL_CURRENT_COLOR);*/
    	renderSolid(1.0, 0.3, _size, _quadratic);
    	//glCallList(_displayListId);
    	break;
    case Render::WIRE:
    	renderWire(width, _size);
    }
}
