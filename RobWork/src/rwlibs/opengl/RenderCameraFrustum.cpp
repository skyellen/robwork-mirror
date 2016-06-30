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



#include "RenderCameraFrustum.hpp"

#include <rw/math/Constants.hpp>
#include <rwlibs/os/rwgl.hpp>

using namespace rw::math;
using namespace rwlibs::opengl;

RenderCameraFrustum::RenderCameraFrustum()
{
    setPerspective(640.0/480.0, 45.0, 0.01, 2.0);
}

RenderCameraFrustum::~RenderCameraFrustum()
{

}

void RenderCameraFrustum::setPerspective(double aspect, double fovy_deg, double vnear, double depth){
    // the draw type has no effect on Rendering of lines
    // calculate  aspect and fovy
    double fovy = fovy_deg * Deg2Rad * 0.5;
    _z = (float)-depth;
    _y = (float)(tan(fovy)*_z);
    _x = (float)(_y*aspect);
    _vnear = (float)vnear;
    _ynear = (float)(tan(fovy)*vnear);
    _xnear = (float)(_ynear*aspect);
}

void RenderCameraFrustum::draw(const rw::graphics::DrawableNode::RenderInfo& info,rw::graphics::DrawableNode::DrawType type, double alpha) const {
    //std::cout << x << " " << y << std::endl;
    // and now draw it
    glDisable(GL_LIGHTING);
    glPushAttrib(GL_CURRENT_BIT);
    glColor4f(0.5,0.5,0.5,1.0);
    glBegin(GL_LINES);
    {
        glVertex4f(  0 ,  0 ,  0 , 0.5);
        glVertex4f(  _x ,  _y ,  _z , 0.5);
        glVertex4f(  0 ,  0 ,  0 , 0.5);
        glVertex4f(  _x , -_y ,  _z , 0.5);
        glVertex4f(  0 ,  0 ,  0 , 0.5);
        glVertex4f( -_x ,  _y ,  _z , 0.5);
        glVertex4f(  0 ,  0 ,  0 , 0.5);
        glVertex4f( -_x , -_y ,  _z , 0.5);

        glVertex4f(  _x ,  _y ,  _z , 0.5);
        glVertex4f( -_x ,  _y ,  _z , 0.5);
        glVertex4f(  _x , -_y ,  _z , 0.5);
        glVertex4f(  _x ,  _y ,  _z , 0.5);

        glVertex4f(  _x ,  -_y ,  _z , 0.5);
        glVertex4f( -_x ,  -_y ,  _z , 0.5);
        glVertex4f( -_x ,  -_y ,  _z , 0.5);
        glVertex4f( -_x ,   _y ,  _z , 0.5);

        // draw near clip
        glVertex4f(  _xnear ,  _ynear ,  -_vnear , 0.5);
        glVertex4f( -_xnear ,  _ynear ,  -_vnear , 0.5);
        glVertex4f(  _xnear , -_ynear ,  -_vnear , 0.5);
        glVertex4f(  _xnear ,  _ynear ,  -_vnear , 0.5);

        glVertex4f(  _xnear ,  -_ynear ,  -_vnear , 0.5);
        glVertex4f( -_xnear ,  -_ynear ,  -_vnear , 0.5);
        glVertex4f( -_xnear ,  -_ynear ,  -_vnear , 0.5);
        glVertex4f( -_xnear ,   _ynear ,  -_vnear , 0.5);
    }
    glEnd();
    glPopAttrib();
    glEnable(GL_LIGHTING);
}

