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

#include <rw/common/Ptr.hpp>

#include <list>
#include <string>

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
    	//! @brief Policy for adding parent nodes.
        typedef enum {
        	Front,//!< Add to front of list.
        	Back  //!< Add to back of list (default).
        } AddPolicy;

        //! @brief Smart pointer type for SceneNode.
        typedef rw::common::Ptr<SceneNode> Ptr;

        //! @brief Type for the list of nodes.
        typedef std::list<SceneNode::Ptr> NodeList;

        //! @brief Type for a const list of nodes.
        typedef const std::list<SceneNode::Ptr> NodeListConst;

        //! @brief Node types.
        enum NodeType {
        	GroupType=0,      //!< For a GroupNode.
			CameraType,       //!< For a SceneCamera.
			DrawableType,     //!< For a DrawableNode.
			UserBeginType=1024//!< For user defined types.
        };

    private:
        SceneNode();
        SceneNode(const SceneNode&);
        SceneNode& operator=(const SceneNode&);

    protected:
        /**
         * @brief Construct new scene node.
         * @param name [in] name of the node.
         * @param type [in] the NodeType.
         */
        SceneNode(const std::string& name, int type):_name(name),_type(type){}

    public:
        //! @brief Destructor.
        virtual ~SceneNode(){}

        /*void setParent(SceneNode::Ptr node){
            _parent = node;
        }*/

        /**
         * @brief Add a parent node.
         * @param node [in] the node to add as parent node.
         * @param policy [in] the AddPolicy (default is Back).
         */
        void addParent(SceneNode::Ptr node, AddPolicy policy=Back){
            if(!hasParent(node)){
                if(policy==Back){
                    _parentNodes.push_back(node);
                } else {
                    _parentNodes.push_front(node);
                }
            }
        }

        /**
         * @brief Check if the given node is a parent node.
         * @param parent [in] the node to look for.
         * @return true if \b parent is a parent node.
         */
        bool hasParent(SceneNode::Ptr parent){
            std::list<SceneNode::Ptr >::iterator location =
              std::find(_parentNodes.begin(), _parentNodes.end(), parent);
            return location !=_parentNodes.end();
        }

        /**
         * @brief erases the parent from the parent list.
         * @note this node is not removed from the parents child list, using this
         * @param node [in] the parent to remove.
         */
        void removeParent(SceneNode::Ptr node){
            std::list<SceneNode::Ptr >::iterator location =
              std::find(_parentNodes.begin(), _parentNodes.end(), node);

            if(location!=_parentNodes.end())
                _parentNodes.erase(location);
        }

        /**
         * @brief Remove a parent node.
         * @param node [in] the node.
         * @param parent [in] the parent node to remove as a parent of \b node.
         */
        static void removeParent(SceneNode::Ptr node, SceneNode::Ptr parent){
            node->removeParent(parent);
        }

    public:
        /**
         * @brief Get a pointer to a GroupNode, if this is a GroupNode.
         * @return a pointer, or NULL if the SceneNode is not of correct type.
         */
        virtual GroupNode* asGroupNode(){ return NULL; }

        /**
         * @brief Get a pointer to a CameraNode, if this is a CameraNode.
         * @return a pointer, or NULL if the SceneNode is not of correct type.
         */
        virtual SceneCamera* asCameraNode(){ return NULL; }

        /**
         * @brief Get a pointer to a DrawableNode, if this is a DrawableNode.
         * @return a pointer, or NULL if the SceneNode is not of correct type.
         */
        virtual DrawableNode* asDrawableNode(){ return NULL; }

        /**
         * @brief Get the NodeType.
         * @return the type of node.
         */
        int getType(){ return _type; }

        /**
         * @brief Get the name of the node.
         * @return the name.
         */
        const std::string& getName(){ return _name; }

        /**
         * @brief Set the name of this node.
         * @param name [in] a new name.
         */
        void setName(const std::string& name){ _name = name; }

        //! @brief The name of the node.
        std::string _name;

        //! @brief The list of parent nodes.
        std::list<SceneNode::Ptr> _parentNodes;

        //! @brief The NodeType.
        int _type;
    };

}
}

#endif /* SCENENODE_HPP_ */
