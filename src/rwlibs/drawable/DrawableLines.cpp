#include "DrawableLines.hpp"

using namespace rw::math;
using namespace rwlibs::drawable;

DrawableLines::DrawableLines()
{   
    initialize();
}

DrawableLines::DrawableLines(LineList& lines):
    _lines(lines)
{    
    initialize();
}

void DrawableLines::initialize() {
    _r = 1;
    _g = 0;
    _b = 0;
    _alpha = 1;
    _thickness = 2;
}

DrawableLines::~DrawableLines()
{
}


void DrawableLines::addLine(const Vector3D<>& start, const Vector3D<>& end) {
    _lines.push_back(Line(start, end));
    update(CUSTOM);
}


void DrawableLines::addLines(const LineList& lines) {
    for (LineList::const_iterator it = lines.begin(); it != lines.end(); ++it) {
        _lines.push_back(*it);
    }
    update(CUSTOM);
}

void DrawableLines::setColor(float r, float g, float b, float alpha) {
    _r = r;
    _g = g;
    _b = b;
    _alpha = alpha;
    update(CUSTOM);
}

void DrawableLines::setThickness(float thickness) {
    _thickness = thickness;
    update(CUSTOM);
}

void DrawableLines::clear() {
    _lines.clear();
    update(CUSTOM);
}

void DrawableLines::update(UpdateType type) {    
    if (_displayListId != 0) {
        glDeleteLists(_displayListId, 1);
    }
    
    _displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPushMatrix();
    glColor4f(_r, _g, _b, _alpha);
    glLineWidth(_thickness);
    
    glBegin(GL_LINES);
    // Draw all faces.
    for (LineList::iterator it = _lines.begin(); it != _lines.end(); ++it) {
        glVertex3f((*it).first(0),(*it).first(1),(*it).first(2));
        glVertex3f((*it).second(0),(*it).second(1),(*it).second(2));
            }        
    
    glEnd();
        
    glPopMatrix();
    glEndList();    
}

