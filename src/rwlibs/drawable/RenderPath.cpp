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


#include "RenderPath.hpp"

using namespace rwlibs::drawable;
using namespace rw

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
