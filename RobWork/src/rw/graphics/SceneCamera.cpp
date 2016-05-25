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

#include "SceneCamera.hpp"
#include "SceneNode.hpp"

using namespace rw::graphics;
using namespace rw::math;

SceneCamera::SceneCamera(const std::string& name, SceneNode::Ptr subGraph):
        SceneNode("Camera", SceneNode::CameraType),
    _x(0),_y(0),_w(640),_h(480),
    _drawMask(DrawableNode::ALL ),
    _clearMask(1),
    _depthTestEnabled(true),
    _lightningEnabled(true),
    _clearBufferEnabled(false),
    _enabled(true),
    _subGraph(subGraph),
    _name(name),
    _ratioControl(Auto)
{
    _aspectRatio = _w / static_cast<double>(_h);
    _pmatrix.setPerspective(45, _w / static_cast<double>(_h), 0.1, 30);
}

SceneCamera::~SceneCamera(){

}

// Projection matrix stuff
void SceneCamera::setPerspective(double fov, int w, int h, double zNear, double zFar){
    _aspectRatio = ((double)w)/(double)h;
    _pmatrix.setPerspective(fov, _aspectRatio, zNear, zFar);
}

ProjectionMatrix SceneCamera::getProjectionMatrix(){
    return _pmatrix;
}

// Transformation matrix stuff

void SceneCamera::setViewport (int x, int y, int width, int height){
    // when the view port is updated it is important that the projection is also updated if
    // its control is set too auto
    double fov, aspect, zNear, zFar, left, right, bottom, top;
    if(_ratioControl==SceneCamera::Auto){
        if(_pmatrix.getPerspective(fov, aspect, zNear, zFar) ){
            _pmatrix.setPerspective(fov, width/(double)height, zNear, zFar);
        } else if( _pmatrix.getOrtho(left, right, bottom, top, zNear, zFar) ){
            _pmatrix.setOrtho(x, x+width, y, y+height, zNear, zFar);
        } else {
            //for(int i=0;i<4;i++)
            //    for(int j=0;j<4;j++)
            //        std::cout << _pmatrix(i,j) << " ";
            // std::cout << "NOTHING\n";
        }
    }

    if( _ratioControl==SceneCamera::Fixed){

    } else {
        _x = x;
        _y = y;
        _w = width;
        _h = height;
    }
}

void SceneCamera::getViewport (int &x, int &y, int &width, int &height){
    x = _x;
    y = _y;
    width = _w;
    height = _h;
}

void SceneCamera::setDrawMask(int mask){
    _drawMask = mask;
}

int SceneCamera::getDrawMask(){
    return _drawMask;
}

void SceneCamera::setProjectionMatrix( const rw::math::ProjectionMatrix& matrix ){
    _pmatrix = matrix;
    double fov, aspect, zNear, zFar, left, right, bottom, top;
    if(_pmatrix.getPerspective( fov, aspect, zNear, zFar ) ){
        _aspectRatio = aspect;
    } else if(_pmatrix.getOrtho(left, right, bottom, top, zNear, zFar) ) {
        _aspectRatio = (right-left)/(top-bottom);
    } else {
        _aspectRatio = 1;
    }
}

