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

#include "GroupNode.hpp"

using namespace rw::graphics;

GroupNode::GroupNode(const std::string& name):SceneNode(name,SceneNode::GroupType){
    _t3d = rw::math::Transform3D<>::identity();
}

GroupNode* GroupNode::asGroupNode(){
    return this;
}

size_t GroupNode::nrOfChildren(){
    return _childNodes.size();
}

bool GroupNode::hasChild(SceneNode::Ptr child){
    std::list<SceneNode::Ptr >::iterator location =
      std::find(_childNodes.begin(), _childNodes.end(), child);
    return location !=_childNodes.end();
}

void GroupNode::removeChild(SceneNode::Ptr child){
    std::list<SceneNode::Ptr >::iterator location = std::find(_childNodes.begin(), _childNodes.end(), child);

    if(location!=_childNodes.end()){
        // 1. remove this from the childs parent list
        removeParent( child, this );
        // 2. erase the child from the child list
        _childNodes.erase(location);
    }
}

void GroupNode::addChild(SceneNode::Ptr node, AddPolicy policy){
    if(!hasChild(node)){
        if(policy==Back){
            _childNodes.push_back(node);
        } else {
            _childNodes.push_front(node);
        }
    }
}

void GroupNode::addChild(SceneNode::Ptr child, GroupNode::Ptr parent, AddPolicy policy){
    RW_ASSERT(child!=NULL);
    RW_ASSERT(parent!=NULL);

    parent->addChild(child, policy);
    if(!child->hasParent(parent))
        child->addParent(parent);
}

void GroupNode::setTransform(const rw::math::Transform3D<>& t3d){
    _t3d = t3d;
}

rw::math::Transform3D<> GroupNode::getTransform(){
    return _t3d;
}

