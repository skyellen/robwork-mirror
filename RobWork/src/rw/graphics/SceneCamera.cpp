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
    _drawMask(DrawableNode::Physical || DrawableNode::Virtual || DrawableNode::DrawableObject),
    _clearBufferEnabled(false),
    _subGraph(subGraph),
    _enabled(true),
    _name(name)
{
    _pmatrix.setPerspective(45, _w/_h, 0.1, 30);
}

SceneCamera::~SceneCamera(){

}

// Projection matrix stuff
void SceneCamera::setPerspective(double fov, int w, int h, double zNear, double zFar){
    _pmatrix.setPerspective(fov, ((double)w)/(double)h, zNear, zFar);
}

ProjectionMatrix SceneCamera::getProjectionMatrix(){ return _pmatrix; }

// Transformation matrix stuff

void SceneCamera::setViewport (int x, int y, int width, int height){
    _x = x;
    _y = y;
    _w = width;
    _h = height;
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

