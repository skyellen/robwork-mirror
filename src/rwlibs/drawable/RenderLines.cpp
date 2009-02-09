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


#include "RenderLines.hpp"

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rwlibs::drawable;

namespace
{
    void drawVector3D(const Vector3D<>& vec)
    {
        glVertex3f(
            static_cast<float>(vec(0)),
            static_cast<float>(vec(1)),
            static_cast<float>(vec(2)));
    }

	void drawLines(RenderLines::LineList& lines)
    {
	    glBegin(GL_LINES);
	    // Draw all faces.
        BOOST_FOREACH(const RenderLines::Line& line, lines) {
	    	// TODO: better to use glVertex3fv
            drawVector3D(line.first);
            drawVector3D(line.second);
		}
	    glEnd();
	}
}

RenderLines::RenderLines():
	_r(1), _g(0), _b(0),
    _alpha(1),
    _thickness(2)

{
	_displayListId = glGenLists(1);
	rerender();
}

RenderLines::RenderLines(const LineList& lines):
	_lines(lines),
	_r(1), _g(0), _b(0),
    _alpha(1),
    _thickness(2)
{
	_displayListId = glGenLists(1);
	rerender();
}

RenderLines::~RenderLines()
{
	glDeleteLists(_displayListId, 1);
}

void RenderLines::draw(DrawType type, double alpha) const{
	// the draw type has no effect on Rendering of lines
	glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

	glColor4f(_r, _g, _b, _alpha);
    glLineWidth(_thickness);
    glCallList(_displayListId);

    glPopAttrib();
}

void RenderLines::rerender() {
    glNewList(_displayListId, GL_COMPILE);
    drawLines(_lines);
    glEndList();
}

void RenderLines::addLine(const Vector3D<>& start, const Vector3D<>& end)
{
    _lines.push_back(Line(start, end));
    rerender();
}

void RenderLines::addLines(const LineList& lines) {
    for (LineList::const_iterator it = lines.begin(); it != lines.end(); ++it) {
        _lines.push_back(*it);
    }
    rerender();
}

void RenderLines::setColor(float r, float g, float b, float alpha) {
    _r = r;
    _g = g;
    _b = b;
    _alpha = alpha;
}

void RenderLines::setThickness(float thickness) {
    _thickness = thickness;
}

void RenderLines::clear() {
    _lines.clear();
    rerender();
}
