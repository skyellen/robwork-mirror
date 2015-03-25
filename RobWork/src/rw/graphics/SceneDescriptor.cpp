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

#include "SceneDescriptor.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>
#include <rw/graphics/DrawableNode.hpp>

#include <boost/foreach.hpp>

#include <vector>
#include <stack>

using namespace rw::math;
using namespace rw::graphics;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::sensor;

//----------------------------------------------------------------------------


SceneDescriptor::SceneDescriptor()
{
}

SceneDescriptor::~SceneDescriptor()
{
}

void SceneDescriptor::setVisible( bool visible, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return;
    _frameStateMap[f].visible = visible;
    BOOST_FOREACH(DrawableProxy::Ptr d, _frameStateMap[f].drawables){
        d->visible = visible;
    }
}

bool SceneDescriptor::isVisible(rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return false;
    return _frameStateMap[f].visible;
}

void SceneDescriptor::setHighlighted( bool highlighted, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return;

    _frameStateMap[f].highlighted = highlighted;

    BOOST_FOREACH(DrawableProxy::Ptr d, _frameStateMap[f].drawables){
        d->highlighted = highlighted;
    }
}

bool SceneDescriptor::isHighlighted( rw::kinematics::Frame* f) {
    return _frameStateMap[f].highlighted;
}

void SceneDescriptor::setFrameAxisVisible( bool visible, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end()){
        return;
    }
    _frameStateMap[f].frameAxisVisible = visible;
}

bool SceneDescriptor::isFrameAxisVisible( rw::kinematics::Frame* f) {
    if(_frameStateMap.find(f)==_frameStateMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].frameAxisVisible;
}

void SceneDescriptor::setDrawType( DrawableNode::DrawType type, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return;
    _frameStateMap[f].dtype = type;
    BOOST_FOREACH(DrawableProxy::Ptr d, _frameStateMap[f].drawables){
        d->dtype = type;
    }
}

DrawableNode::DrawType SceneDescriptor::getDrawType( rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].dtype;
}

void SceneDescriptor::setDrawMask( unsigned int mask, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        RW_THROW("Frame is not in the scene!");
    _frameStateMap[f].dmask = mask;
    BOOST_FOREACH(DrawableProxy::Ptr d, _frameStateMap[f].drawables){
        d->dmask = mask;
    }
}

unsigned int SceneDescriptor::getDrawMask(rw::kinematics::Frame* f ){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].dmask;
}


void SceneDescriptor::setTransparency( double alpha, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return;

    _frameStateMap[f].alpha = alpha;
    BOOST_FOREACH(DrawableProxy::Ptr d, _frameStateMap[f].drawables){
        d->alpha = alpha;
    }
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame* frame){
    // add frame to frame map
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = drawable->getName();
    //proxy->dmask = drawable->dmask;
    proxy->dnode = drawable;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addDrawable(const std::string& filename, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = filename;
    proxy->dmask = dmask;
    proxy->filename = filename;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->frameSize = size;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addGeometry(const std::string& name,rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->dmask = dmask;
    proxy->geom = geom;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addModel3D(const std::string& name, Model3D::Ptr model, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->dmask = dmask;
    proxy->model = model;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addImage(const std::string& name, rw::sensor::Image::Ptr img, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->dmask = dmask;
    proxy->img = img;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addScan(const std::string& name, rw::geometry::PointCloud::Ptr scan, rw::kinematics::Frame* frame, int dmask){
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->dmask = dmask;
    proxy->scan25 = scan;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

/*
SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addLines(const std::string& name,const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask){
    DrawableGeometryNode::Ptr drawable = _scene->makeDrawable(name, lines);
    if(drawable==NULL)
        return drawable;
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

*/

SceneDescriptor::DrawableProxy::Ptr SceneDescriptor::addRender(const std::string& name,rw::graphics::Render::Ptr render, rw::kinematics::Frame* frame, int dmask) {
    DrawableProxy::Ptr proxy = ownedPtr( new DrawableProxy());
    proxy->name = name;
    proxy->dmask = dmask;
    proxy->render = render;

    _frameStateMap[frame].drawables.push_back(proxy);
    return proxy;
}

/*
std::vector<SceneDescriptor::DrawableProxy::Ptr> SceneDescriptor::getDrawables(rw::kinematics::Frame* f){
    return _frameStateMap[f].drawables;
}

void SceneDescriptor::findDrawable(const std::string& name, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return NULL;
    return _scene->findDrawable(name, _frameStateMap[f]);
}

bool SceneDescriptor::removeDrawable(DrawableNode::Ptr drawable){
    return _scene->removeDrawable(drawable);
}

bool SceneDescriptor::removeDrawables(rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return false;
    return _scene->removeDrawables(_frameStateMap[f]);
}

bool SceneDescriptor::removeDrawable(const std::string& name){
    return _scene->removeDrawable(name);
}

bool SceneDescriptor::removeDrawables(const std::string& name){
    return _scene->removeDrawables(name);
}

bool SceneDescriptor::removeDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return false;
    return _scene->removeDrawable(drawable, _frameStateMap[f]);
}

bool SceneDescriptor::removeDrawable(const std::string& name, rw::kinematics::Frame* f){
    if(_frameStateMap.find(f)==_frameStateMap.end())
        return false;
    return _scene->removeChild(name, _frameStateMap[f]);
}


rw::kinematics::Frame* SceneDescriptor::getFrame(DrawableNode::Ptr d){
    //std::cout << d->_parentNodes.size() << std::endl;
    GroupNode::Ptr gn = d->_parentNodes.front().cast<GroupNode>();
    if(gn==NULL)
        return NULL;
    return _nodeFrameMap[gn];
}
*/
