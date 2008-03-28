#include "RenderPath.hpp"

RenderPath::RenderPath(const std::string& filename)
{
    std::ifstream in(filename.c_str());
    
    while (!in.eof()) {
        Vector3D<> p;
        in >> p(0);
        in >> p(1);
        in >> p(2);
        _points.push_back(p);
    }
    in.close();
}

RenderPath::~RenderPath()
{
}


void RenderPath::update(UpdateType type) {
    if (_displayListId != 0) {
        glDeleteLists(_displayListId, 1);
    }

    _displayListId = glGenLists(1);
    glNewList(_displayListId, GL_COMPILE);
    glPushMatrix();
    glColor4f(_r, _g, _b, _alpha);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
            
    for (std::vector<Vector3D<> >::interator it = _points.begin(); it != points.end(); ++it) {
        glVertex3d((*it)(0), (*it)(1), (*it)(2));
    } 

    glEnd();
    
    glPopMatrix();
    glEndList();

    
}