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
	    glPopAttrib();
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
    glColor4f(_r, _g, _b, _alpha);
    glLineWidth(_thickness);
    glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
    glCallList(_displayListId);
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
