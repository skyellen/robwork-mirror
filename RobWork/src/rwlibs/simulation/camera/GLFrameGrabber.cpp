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


#include "GLFrameGrabber.hpp"
#include <rw/common/Log.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <cmath>

using namespace rw::common;
using namespace rw::graphics;
using namespace rwlibs::simulation;



#define USE_FRAMEBUFFERS

#ifndef USE_FRAMEBUFFERS

GLFrameGrabber::GLFrameGrabber(
    int width, int height, double fov,
    rw::graphics::WorkCellGLDrawer *drawer)
    :
    FrameGrabber(width,height,rw::sensor::Image::RGB),
    _fieldOfView(fov),_drawer(drawer),
    _perspTrans(rw::math::Transform3D<double>::identity())
{}

GLFrameGrabber::~GLFrameGrabber(){}

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
    char *imgData = _img->getImageData();



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

    // the image is mirrored in the x-axis
    for(size_t y=0;y<_img->getHeight();y++){
        for(size_t x=0;x<_img->getWidth()/2;x++){
            int idx = (y*_img->getWidth()+x)*3;
            int idxback = (y*_img->getWidth()+_img->getWidth()-1-x)*3;
            for(size_t c=0;c<3;c++){
                unsigned char tmp = imgData[idx+c];
                imgData[idx+c] = imgData[idxback+c];
                imgData[idxback+c] = tmp;
            }
        }
    }
}
#else


GLFrameGrabber::GLFrameGrabber(int width, int height, double fov,double near, double far)
    :
    FrameGrabber(width,height,rw::sensor::Image::RGB),
    _fieldOfView(fov),_drawer(NULL),
    _perspTrans(rw::math::Transform3D<double>::identity()),
    _near(near),_far(far)
{

/*
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
	switch(status){
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		std::cout<<"good" << std::endl;
		break;
	default:
		std::cout << "NOT GOOD AT ALLL, status: " << status << std::endl;;
	}

	std::cout << _fbId << " " << _renderId << std::endl;

    RWGLFrameBuffer::test(Log::infoLog());

    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
*/
}

GLFrameGrabber::~GLFrameGrabber(){
    //RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
    //RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
    //RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
}
void GLFrameGrabber::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state) {
    rw::math::Transform3D<> wTf = rw::kinematics::Kinematics::worldTframe(frame, state);
    _view->_viewCamera->setTransform( wTf );
    _drawer->renderView(_view);


    /*
    glPushMatrix();
    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // change viewport to the width and height of image
    GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
    glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
    glViewport(0,0,_img->getWidth(),_img->getHeight()); // set camera view port
    // set camera perspective in relation to a camera model


	//glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);

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
    //_drawer->drawCameraView(state, frame);



    // copy rendered scene to image
    char *imgData = _img->getImageData();
    //glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);

    //glBindTexture(GL_TEXTURE_2D, textureId);

    glReadPixels(
        0, 0,
        _img->getWidth(), _img->getHeight(),
        GL_RGB, GL_UNSIGNED_BYTE, imgData);
    //glBindTexture(GL_TEXTURE_2D, 0);



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

    // the image is mirrored in the x-axis
    for(size_t y=0;y<_img->getHeight();y++){
        for(size_t x=0;x<_img->getWidth()/2;x++){
            for(size_t c=0;c<3;c++){
                int idx = (y*_img->getWidth()+x)*3;
                int idxback = (y*_img->getWidth()+_img->getWidth()-1-x)*3;
                unsigned char tmp = imgData[idx+c];
                imgData[idx+c] = imgData[idxback+c];
                imgData[idxback+c] = tmp;
            }
        }
    }
    */

}


#endif

void GLFrameGrabber::resize(int width, int height) {
    FrameGrabber::resize(width,height);

    _view->_camGroup->setOffscreenRenderColor(_colorCode);
    _view->_camGroup->setOffscreenRenderSize(getWidth(),getHeight());
    _view->_camGroup->setCopyToImage(_img);
};

void GLFrameGrabber::resize(int width, int height, rw::sensor::Image::ColorCode colorCode)
{
    FrameGrabber::resize(width,height, colorCode);
    _view->_camGroup->setOffscreenRenderColor(_colorCode);
    _view->_camGroup->setOffscreenRenderSize(getWidth(),getHeight());
    _view->_camGroup->setCopyToImage( _img);
};


void GLFrameGrabber::init(rw::graphics::SceneViewer::Ptr drawer){
    _drawer = drawer;
    std::cout << "initialize glframegrabber";
    SceneViewer::View::Ptr view = _drawer->createView("CameraSensorView");

    view->_viewCamera->setAspectRatioControl(SceneCamera::Scale);
    view->_viewCamera->setEnabled(true);
    view->_viewCamera->setClearBufferEnabled(true);
    view->_viewCamera->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    view->_viewCamera->setDepthTestEnabled( true );
    view->_viewCamera->setLightningEnabled( true );
    view->_viewCamera->setRefNode( drawer->getScene()->getRoot() );
    //std::cout << width <<  " " << height << std::endl;
    view->_viewCamera->setPerspective(_fieldOfView, getWidth(), getHeight(), _near, _far);
    view->_viewCamera->setViewport(0,0, getWidth(), getHeight());
    view->_viewCamera->setAspectRatioControl(SceneCamera::Fixed);
    view->_viewCamera->attachTo( drawer->getMainView()->_viewCamera->getRefNode() );
    view->_viewCamera->setDrawMask(DrawableNode::Physical);
    // render offscreen
    view->_camGroup->setOffscreenRenderEnabled(true);
    view->_camGroup->setOffscreenRenderColor(_colorCode);
    view->_camGroup->setOffscreenRenderSize(getWidth(),getHeight());
    view->_camGroup->setCopyToImage( _img);
    view->_camGroup->setEnabled(true);
    _view = view;

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
