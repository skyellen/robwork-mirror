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


#include "GLFrameGrabber25D.hpp"
#include "RWGLFrameBuffer.hpp"

#include <rw/common/Log.hpp>
#include <rwlibs/os/rwgl.hpp>

#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>

#include <cmath>

using namespace rw::math;
using namespace rw::common;
using namespace rwlibs::simulation;
using namespace rwlibs::drawable;

GLFrameGrabber25D::GLFrameGrabber25D(int width,
                                     int height,
                                     double fov,
                                     rwlibs::drawable::WorkCellGLDrawer *drawer):
    FrameGrabber25D(width, height),
    _fieldOfView(fov),
    _drawer(drawer),
    _perspTrans(rw::math::Transform3D<double>::identity()),
    _minDepth(0.25),
    _maxDepth(15)

{
    RWGLFrameBuffer::initialize();

    _fbId = 0;
    _renderId = 0;
    RWGLFrameBuffer::glGenFramebuffersEXT(1, &_fbId);
    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);

    RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderId);
    // select render
    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);
    // create render storage
    RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB8, width, height);
    //Attach color buffer to FBO
    RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, _renderId);

    // now if we need depth of image we also attach depth render buffer
    RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderDepthId);
    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderDepthId);
    RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, width, height);
     //Attach depth buffer to FBO
    RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, _renderDepthId);

    //Does the GPU support current FBO configuration?
    GLenum status;
    status = RWGLFrameBuffer::glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT)
        RW_THROW("Failed to initialize FrameBuffers with status "<<status);

    RWGLFrameBuffer::test(Log::infoLog());

    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

GLFrameGrabber25D::~GLFrameGrabber25D() {
    RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
}


void GLFrameGrabber25D::setMaxDepth(double depth){
	if(depth<_minDepth)
		RW_THROW("MaxDepth not allowed to be smaller than MinDepth: "<< depth <<">" <<_minDepth );
	_maxDepth = depth;
}

void GLFrameGrabber25D::setMinDepth(double depth){
	if(depth<_maxDepth)
		RW_THROW("MinDepth not allowed to be larger than MaxDepth: "<< depth <<"<" <<_maxDepth );
	_minDepth = depth;
}





void GLFrameGrabber25D::grab(rw::kinematics::Frame *frame,
                             const rw::kinematics::State& state,
                             std::vector<rw::math::Vector3D<float> >* result){
    glPushMatrix();

    if(_depthData.size() != getWidth()*getHeight() )
            _depthData.resize(getWidth()*getHeight());


    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,getWidth(),getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


    glMatrixMode(GL_PROJECTION);
    {
        glPushMatrix();
        glLoadIdentity();
        GLdouble aspect = (GLdouble)getWidth() / (GLdouble)getHeight();
        gluPerspective((GLdouble)_fieldOfView, aspect, (GLdouble)0.1, (GLdouble)100);
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // we rotate because glReadPixels put the char array in different order
    glRotated(180,0,0,1);
    // render scene
    _drawer->drawCameraView(state, frame);


   glReadPixels(
        0, 0,
        getWidth(), getHeight(),
        GL_DEPTH_COMPONENT, GL_FLOAT, &_depthData[0]);


    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );


    // now unproject all pixel values
    if (result != NULL && result->size() != getHeight()*getWidth())
        result->resize(getHeight()*getWidth());

    for(size_t y=0;y<getHeight();y++){
        for(size_t x=0;x<getWidth();x++){
            double winX=x,winY=y,winZ=_depthData[x+y*getWidth()];
            double posX, posY, posZ;
            gluUnProject( winX, winY, winZ,
                    modelview, projection, viewport,
                    &posX, &posY, &posZ);
            if (result != NULL) {
                Vector3D<float>& q = (*result)[x+y*getWidth()];
                q(0) = (float)posX;
                q(1) = (float)posY;
                q(2) = (float)posZ;
            }
        }
    }

    // change viewport settings back
    glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port

    glMatrixMode(GL_PROJECTION);
    {
        glPopMatrix();
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    glPopMatrix();

}


