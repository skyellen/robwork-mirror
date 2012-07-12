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

#include <rw/common/Log.hpp>
#include <rwlibs/os/rwgl.hpp>
#include <rw/math/Math.hpp>
#include <rwlibs/opengl/RWGLFrameBuffer.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <cmath>

using namespace rw::math;
using namespace rw::common;
using namespace rw::graphics;
using namespace rwlibs::simulation;
using namespace rwlibs::opengl;


GLFrameGrabber25D::GLFrameGrabber25D(int width,
                                     int height,
                                     double fov,
                                     double near, double far):
    FrameGrabber25D(width, height),
    _fieldOfView(fov),
    _drawer(NULL),
    _perspTrans(rw::math::Transform3D<double>::identity()),
    _minDepth(near),
    _maxDepth(far)
{
}

GLFrameGrabber25D::~GLFrameGrabber25D() {
}

void GLFrameGrabber25D::init(rw::graphics::SceneViewer::Ptr drawer){
    _drawer = drawer;
    //std::cout << "initialize GLFrameGrabber25D";
    SceneViewer::View::Ptr view = _drawer->createView("Camera25DSensorView");

    view->_viewCamera->setAspectRatioControl(SceneCamera::Scale);
    view->_viewCamera->setEnabled(true);
    view->_viewCamera->setClearBufferEnabled(true);
    view->_viewCamera->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    view->_viewCamera->setDepthTestEnabled( true );
    view->_viewCamera->setLightningEnabled( true );
    view->_viewCamera->setRefNode( drawer->getScene()->getRoot() );
    //std::cout << width <<  " " << height << std::endl;
    view->_viewCamera->setPerspective(_fieldOfView, getWidth(), getHeight(), _minDepth, _maxDepth);
    view->_viewCamera->setViewport(0,0, getWidth(), getHeight());
    view->_viewCamera->setAspectRatioControl(SceneCamera::Fixed);
    view->_viewCamera->attachTo( drawer->getMainView()->_viewCamera->getRefNode() );
    view->_viewCamera->setDrawMask(DrawableNode::Physical);
    // render offscreen
    view->_camGroup->setOffscreenRenderEnabled(true);
    view->_camGroup->setOffscreenRenderColor(rw::sensor::Image::RGB);
    view->_camGroup->setOffscreenRenderSize(getWidth(),getHeight());
    view->_camGroup->setCopyToScan25D( _img );
    view->_camGroup->setEnabled(true);
    _view = view;
}

void GLFrameGrabber25D::setMaxDepth(double depth){
	if(depth<_minDepth)
		RW_THROW("MaxDepth not allowed to be smaller than MinDepth: "<< depth <<">" <<_minDepth );
	_maxDepth = depth;
    if(_view!=NULL)
        _view->_viewCamera->setPerspective(_fieldOfView, getWidth(), getHeight(), _minDepth, _maxDepth);

}

void GLFrameGrabber25D::setMinDepth(double depth){
	if(depth<_maxDepth)
		RW_THROW("MinDepth not allowed to be larger than MaxDepth: "<< depth <<"<" <<_maxDepth );
	_minDepth = depth;
    if(_view!=NULL)
        _view->_viewCamera->setPerspective(_fieldOfView, getWidth(), getHeight(), _minDepth, _maxDepth);

}

void GLFrameGrabber25D::grab(rw::kinematics::Frame *frame,
                             const rw::kinematics::State& state)
{
    rw::math::Transform3D<> wTf = rw::kinematics::Kinematics::worldTframe(frame, state);
    // TODO: we need to transform the image such that the camera looks in  the positive z-direction
    _view->_viewCamera->setTransform( wTf );
    _drawer->renderView(_view);
}


