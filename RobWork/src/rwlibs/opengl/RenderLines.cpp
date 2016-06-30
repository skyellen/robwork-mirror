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



#include "RenderLines.hpp"

#include <rw/geometry/Line.hpp>
#include <rwlibs/os/rwgl.hpp>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rw::geometry;
using namespace rw::graphics;
namespace
{
    void drawVector3D(const Vector3D<>& vec)
    {
        glVertex3f(
            static_cast<float>(vec(0)),
            static_cast<float>(vec(1)),
            static_cast<float>(vec(2)));
    }

	void drawLines(const std::vector<Line>& lines)
    {
	    glBegin(GL_LINES);
	    // Draw all faces.
        BOOST_FOREACH(const Line& line, lines) {
	    	// TODO: better to use glVertex3fv
            drawVector3D(line.p1());
            drawVector3D(line.p2());
		}
	    glEnd();
	}
}

RenderLines::RenderLines():
	_r(1), _g(0), _b(0),
    _alpha(1),
    _thickness(1)

{
	//_displayListId = glGenLists(1);
	//rerender();
}

RenderLines::RenderLines(const std::vector<Line>& lines):
	_lines(lines),
	_r(1), _g(0), _b(0),
    _alpha(1),
    _thickness(1)
{
	//_displayListId = glGenLists(1);
	//rerender();
}

RenderLines::~RenderLines()
{
	//glDeleteLists(_displayListId, 1);
}

void RenderLines::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{
	// the draw type has no effect on Rendering of lines
	glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

	glColor4f(_r, _g, _b, _alpha);
    glLineWidth(_thickness);
    //glCallList(_displayListId);
    drawLines(_lines);
    glPopAttrib();
}

void RenderLines::rerender() {
    //glNewList(_displayListId, GL_COMPILE);
    //drawLines(_lines);
    //glEndList();
}

void RenderLines::addLine(const Vector3D<>& start, const Vector3D<>& end)
{
    _lines.push_back(Line(start, end));
    //rerender();
}

void RenderLines::addLines(const std::vector<Line>& lines) {
    for (std::vector<Line>::const_iterator it = lines.begin(); it != lines.end(); ++it) {
        _lines.push_back(*it);
    }
    //rerender();
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
    //rerender();
}
