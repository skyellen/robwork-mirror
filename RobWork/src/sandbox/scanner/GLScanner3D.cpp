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


#include "GLScanner3D.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cmath>

using namespace rwlibs::simulation;
using namespace rwlibs::drawable;
using namespace rw::kinematics;


void GLScanner3D::update(double dt, const rw::kinematics::State& state){

    Frame* frame = getFrame();
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img.getWidth(),_img.getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)_img.getWidth() / (GLdouble)_img.getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)0.1, (GLdouble)100);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);
    // copy rendered scene to image
    //float *imgData = _img->getImageData();
    glReadPixels(0, 0, _img.getWidth(), _img.getHeight(),
        GL_DEPTH_COMPONENT, GL_FLOAT, &_img.getImageData()[0] );

    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
