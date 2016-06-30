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
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/os/rwgl.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rwlibs::simulation;

GLFrameGrabber::GLFrameGrabber(int width, int height, double fov,double near, double far)
    :
    FrameGrabber(width,height,rw::sensor::Image::RGB),
    _fieldOfView(fov),_drawer(NULL),
    _perspTrans(rw::math::Transform3D<double>::identity()),
    _near(near),_far(far)
{
}

GLFrameGrabber::~GLFrameGrabber(){
}

void GLFrameGrabber::grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state) {

    rw::math::Transform3D<> wTf = rw::kinematics::Kinematics::worldTframe(frame, state);
    _view->_viewCamera->setTransform( wTf );
    // the image is grabbed in the negative z-axis
    _drawer->renderView(_view);

}

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
