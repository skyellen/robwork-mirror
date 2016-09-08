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

using namespace rwlibs::opengl;
using namespace rw::graphics;

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
	}

	void renderSolid(float alpha, float width, float size, GLUquadricObj *quad){
		const float REL_WIDTH = 0.05f;
		const float lenHead = size*3*REL_WIDTH; // the arrow head
		const float widthHead = size*2*REL_WIDTH;
		const float lenBody = size-lenHead; // the arrow body
		const float widthBody = width*size*REL_WIDTH;

        // Nice frame
        // Draw z-axis
        glColor4f(0.0f, 0.0f, 1.0f, alpha); // Blue color
		gluCylinder(quad, widthBody, widthBody, lenBody, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,lenBody);// Center The Cone
        gluCylinder(quad,widthHead,0.0f,lenHead,32,32); // A Cone

        // Draw x-axis
        glColor4f(1.0f, 0.0f, 0.0f, alpha); // Red color
        glTranslatef(0.0f,0.0f,-lenBody); // Center The Cylinder
        glRotatef(90.0f,0.0f,1.0f,0.0f); // Rotate around y-ax
        gluCylinder(quad, widthBody, widthBody, lenBody, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,lenBody);// Center The Cone
        gluCylinder(quad,widthHead,0.0f,lenHead,32,32);// A Cone

        // Draw y-axis
        glColor4f(0.0f, 1.0f, 0.0f, alpha); // Green color
        glTranslatef(0.0f,0.0f,-lenBody);                     // Center The Cylinder
        glRotatef(90.0f,-1.0f,0.0f,0.0f); // Rotate around y-axis
        gluCylinder(quad, widthBody, widthBody, lenBody, 32, 32);    // Draw Our Cylinder
        glTranslatef(0.0f,0.0f,lenBody);// Center The Cone
        gluCylinder(quad, widthHead, 0.0f, lenHead, 32, 32);// A Cone
	}

	void initializeColors(float *r, float *g, float *b){
		r[0] = 1.0; r[1] = 0.0; r[2] = 0.0; r[3] = 1.0;
		g[0] = 0.0; g[1] = 1.0; g[2] = 0.0; g[3] = 1.0;
		b[0] = 0.0; b[1] = 0.0; b[2] = 1.0; b[3] = 1.0;
	}
}

RenderFrame::RenderFrame(float size):
    _size(size),
    _quadratic(NULL)
{
	initializeColors(_red,_green,_blue);
}

RenderFrame::~RenderFrame() {
	if (_quadratic != NULL)
		gluDeleteQuadric(_quadratic);
}

void RenderFrame::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
{
    if(_quadratic==NULL)
        _quadratic = gluNewQuadric();
	const float width = 0.3f;
	_green[3] = (float)alpha;
	_red[3] = (float)alpha;
	_blue[3] = (float)alpha;
    switch (type) {
    case DrawableNode::SOLID:
    case DrawableNode::OUTLINE: // Draw nice frame
    	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    	renderSolid(1.0f, 0.3f, _size, _quadratic);
    	break;
    case DrawableNode::WIRE:
    	renderWire(width, _size);
    	break;
    }
}
