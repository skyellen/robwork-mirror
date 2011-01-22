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

#ifndef RW_GRAPHICS_SCENENODE_HPP_
#define RW_GRAPHICS_SCENENODE_HPP_

#include <rw/math/Transform3D.hpp>
#include <list>

namespace rw {
namespace graphics {

    // forward declarations
    class SceneCamera;
    class GroupNode;
    class DrawableNode;

    /**
     * @brief a node that can have leafs (DrawableNode) or other nodes as children.
     */
    class SceneNode {
    public:
        typedef enum{Front, Back} AddPolicy;

        typedef rw::common::Ptr<SceneNode> Ptr;
        typedef std::list<SceneNode::Ptr> NodeList;
        typedef const std::list<SceneNode::Ptr> NodeListConst;

        //! this is the default nodetype add more
        enum NodeType{GroupType=0,CameraType,DrawableType,UserBeginType=1024};
    private:
        SceneNode();
        SceneNode(const SceneNode&);
        SceneNode& operator=(const SceneNode&);

    protected:
        SceneNode(const std::string& name, int type):_name(name),_type(type){}

    public:

        /*void setParent(SceneNode::Ptr node){
            _parent = node;
        }*/

        void addParent(SceneNode::Ptr node, AddPolicy policy=Back){
            if(!hasParent(node)){
                if(policy==Back){
                    _parentNodes.push_back(node);
                } else {
                    _parentNodes.push_front(node);
                }
            }
        }

        bool hasParent(SceneNode::Ptr parent){
            std::list<SceneNode::Ptr >::iterator location =
              std::find(_parentNodes.begin(), _parentNodes.end(), parent);
            return location !=_parentNodes.end();
        }

        void removeParent(SceneNode::Ptr node){
            std::list<SceneNode::Ptr >::iterator location =
              std::find(_parentNodes.begin(), _parentNodes.end(), node);
            if(location!=_parentNodes.end())
                _parentNodes.erase(location);
        }

        virtual GroupNode* asGroupNode(){ return NULL; }
        virtual SceneCamera* asCameraNode(){ return NULL; }
        virtual DrawableNode* asDrawableNode(){ return NULL; }

        int getType(){ return _type; }
        const std::string& getName(){ return _name; }
        std::string _name;
        std::list<SceneNode::Ptr> _parentNodes;
        //SceneNode::Ptr _parent;
        int _type;
    };

}
}

#endif /* SCENENODE_HPP_ */
