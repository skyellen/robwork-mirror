/*********************************************************************
 * RobWork Version 0.2
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

#include "GLFrameGrabber.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cmath>

using namespace rwlibs::simulation;
using namespace rwlibs::drawable;

void GLFrameGrabber::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state){
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img->getWidth(),_img->getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)_img->getWidth() / (GLdouble)_img->getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)0.1, (GLdouble)100);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);
    // copy rendered scene to image
    unsigned char *imgData = _img->getImageData();
    glReadPixels(
        0, 0,
        _img->getWidth(), _img->getHeight(),
        GL_RGB, GL_UNSIGNED_BYTE, imgData);

    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/*    // Create handle to FrameBuffer
      GLuint fbo;
      glGenFramebuffersEXT(1, &fbo);
      // Bind handle with specific framebuffer
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
      // Create handle to a renderbuffer
      GLuint renderbuffer;
      glGenRenderbuffersEXT(1, &renderbuffer);
      // Bind handle to specific renderbuffer
      glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
      // Allocate storage to the render buffer
      glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
      _img->getWidth(), _img->getHeight());
      // Attach the renderbuffer to the framebuffer
      glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
      GL_RENDERBUFFER_EXT, renderbuffer);
      //

      // Check the state of the framebuffer
      GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);

      // Check how many auxiliary buffers are present (can be used as colour buffers)
      GLuint nr_aux;
      glGetIntegerv(GL_AUX_BUFFERS,&nr_aux);
*/

/*    GLboolean db;
    glGetBooleanv(GL_DOUBLEBUFFER,&db);
    if( db ) std::cout << "DOUBLE BUFFER IS USED..." << std::endl;
    else std::cout << "DOUBLE BUFFER IS NOT USED..." << std::endl;
    GLint buf;
    glGetIntegerv(GL_DRAW_BUFFER, &buf);
    std::cout << "CURRENT BUFFER IS USED: " << buf << std::endl;
    */
