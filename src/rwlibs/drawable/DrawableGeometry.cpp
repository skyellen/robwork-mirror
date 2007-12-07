#include "DrawableGeometry.hpp"

using namespace rw::geometry;
using namespace rwlibs::drawable;

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


DrawableGeometry::DrawableGeometry(Geometry* geometry, float r, float g, float b):
    _geometry(geometry),
    _r(r),
    _g(g),
    _b(b)
{
    update(CUSTOM);
}


DrawableGeometry::~DrawableGeometry() {
    delete _geometry;
}

void DrawableGeometry::setColor(float r, float g, float b) {
    _r = r;
    _g = g;
    _b = b;
}


void DrawableGeometry::update(UpdateType type) {
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
        std::for_each(_geometry->getFaces().begin(), _geometry->getFaces().end(), drawFace);

        glEnd();

        glPopMatrix();
        glEndList();
    }
}
