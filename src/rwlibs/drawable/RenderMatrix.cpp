/*********************************************************************
 * RobWork Version 0.3
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
#include "RenderMatrix.hpp"

#include <iostream>
#include <rw/math/Math.hpp>

using namespace rwlibs::drawable;
using namespace rw::math;

namespace
{
	void drawBox(float x, float y, float x_size, float y_size, float z)
    {
		const float xtmp = x + x_size;
		const float ytmp = y + y_size;

		glBegin(GL_QUADS);
        {
			// Front Face
			glVertex3f(    x,     y,  z);	// Bottom Left Of The Texture and Quad
			glVertex3f( xtmp,     y,  z);	// Bottom Right Of The Texture and Quad
			glVertex3f( xtmp,  ytmp,  z);	// Top Right Of The Texture and Quad
			glVertex3f(    x,  ytmp,  z);	// Top Left Of The Texture and Quad

			// Back Face
			glVertex3f(    x,     y, 0.0f);	// Bottom Right Of The Texture and Quad
			glVertex3f(    x,  ytmp, 0.0f);	// Top Right Of The Texture and Quad
			glVertex3f( xtmp,  ytmp, 0.0f);	// Top Left Of The Texture and Quad
			glVertex3f( xtmp,     y, 0.0f);	// Bottom Left Of The Texture and Quad

			// Top Face
			glVertex3f(    x,  ytmp, 0.0f);	// Top Left Of The Texture and Quad
			glVertex3f(    x,  ytmp,  z);	// Bottom Left Of The Texture and Quad
			glVertex3f( xtmp,  ytmp,  z);	// Bottom Right Of The Texture and Quad
			glVertex3f( xtmp,  ytmp, 0.0f);	// Top Right Of The Texture and Quad

			// Bottom Face
			glVertex3f(    x,     y, 0.0f);	// Top Right Of The Texture and Quad
			glVertex3f( xtmp,     y, 0.0f);	// Top Left Of The Texture and Quad
			glVertex3f( xtmp,     y,  z);	// Bottom Left Of The Texture and Quad
			glVertex3f(    x,     y,  z);	// Bottom Right Of The Texture and Quad

			// Right face
			glVertex3f( xtmp,     y, 0.0f);	// Bottom Right Of The Texture and Quad
			glVertex3f( xtmp,  ytmp, 0.0f);	// Top Right Of The Texture and Quad
			glVertex3f( xtmp,  ytmp,  z);	// Top Left Of The Texture and Quad
			glVertex3f( xtmp,     y,  z);	// Bottom Left Of The Texture and Quad

			// Left Face
			glVertex3f(    x,     y, 0.0f);	// Bottom Left Of The Texture and Quad
			glVertex3f(    x,     y,  z);	// Bottom Right Of The Texture and Quad
			glVertex3f(    x,  ytmp,  z);	// Top Right Of The Texture and Quad
			glVertex3f(    x,  ytmp, 0.0f);	// Top Left Of The Texture and Quad
        }
		glEnd();
	}
}

RenderMatrix::RenderMatrix(
    const std::string& id,
    size_t cols,
    size_t rows,
    float width,
    float height)
    :
	_width(width),_height(height),
	_maxZ(1.0f),_zscale(1.0f),
	_vals(cols,rows)
{}

void RenderMatrix::draw(DrawType type, double alpha) const
{
	// ignores drawstate
	glPushMatrix();

	float x_offset = 0;
    float y_offset = 0;

	for (size_t col = 0; col < _vals.size1(); x_offset += _width) {
		for (size_t row = 0; row < _vals.size2(); y_offset += _height) {
			float zval = Math::clamp(
                _vals(col, row), 0.0f, _maxZ) * _zscale;

			// calculate and draw color
			glColor3f(zval,0.0f,0.0f);

			// draw box
			drawBox(x_offset,y_offset,_width,_height, zval);
		}
	}
	glPopMatrix();
}
