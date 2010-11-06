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


#include "SceneOSG.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <vector>

using namespace rwlibs::drawable;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::models;

SceneOSG::SceneOSG(){}

SceneOSG::~SceneOSG()
{

}


void SceneOSG::draw(DrawableNode::RenderInfo& info){}

void SceneOSG::setWorkCell(rw::models::WorkCell::Ptr wc){ }

rw::models::WorkCell::Ptr SceneOSG::getWorkCell(){}

void SceneOSG::updateSceneGraph(const rw::kinematics::State& state){}

void SceneOSG::setVisible(rw::kinematics::Frame* , bool visible){}

bool SceneOSG::isVisible(rw::kinematics::Frame*){}

void SceneOSG::setHighlighted( rw::kinematics::Frame*, bool highlighted){}

bool SceneOSG::isHighlighted( rw::kinematics::Frame* ) const{

}

void SceneOSG::setFrameAxisVisible( rw::kinematics::Frame*, bool visible){}

bool SceneOSG::isFrameAxisVisible( rw::kinematics::Frame*) const{

}

void SceneOSG::setDrawType( rw::kinematics::Frame*, DrawType type){

}

SceneGraph::DrawType SceneOSG::getDrawType( rw::kinematics::Frame*){

}

void SceneOSG::setDrawMask(rw::kinematics::Frame*, unsigned int mask){

}

unsigned int SceneOSG::getDrawMask(rw::kinematics::Frame*) const{

}


void SceneOSG::setTransparency(double alpha){

}

DrawableNode::Ptr SceneOSG::addFrameAxis(double size, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addGeometry(rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addModel3D(Model3DPtr model, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addImage(const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addScan(const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addScan(const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask){}

DrawableNode::Ptr SceneOSG::addLines(const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask){}


void SceneOSG::addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame*, int dmask){}
std::vector<DrawableNode::Ptr> SceneOSG::getDrawables(){}
std::vector<DrawableNode::Ptr> SceneOSG::getDrawables(rw::kinematics::Frame*){}


DrawableNode::Ptr SceneOSG::findDrawable(const std::string& name){}
std::vector<DrawableNode::Ptr> SceneOSG::findDrawables(const std::string& name){}

bool SceneOSG::removeDrawable(DrawableNode::Ptr drawable){}

bool SceneOSG::removeDrawables(rw::kinematics::Frame*){}

