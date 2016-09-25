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

#ifndef RW_GRAPHICS_GROUPNODE_HPP_
#define RW_GRAPHICS_GROUPNODE_HPP_

#include "SceneNode.hpp"

#include <rw/math/Transform3D.hpp>
#include <list>


namespace rw {
namespace graphics {

    /**
     * @brief a SceneNode that has a transformation and 0 to many children.
     */
    class GroupNode: public SceneNode {
    public:

        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<GroupNode> Ptr;

        //! @brief constructor
        GroupNode(const std::string& name);

        virtual ~GroupNode(){};

        //! @copydoc SceneNode::asGroupNode
        GroupNode* asGroupNode();

        //! @brief get the number of children
        size_t nrOfChildren();

        /**
         * @brief add a child to this group node
         * @param node [in] the child to add
         * @param policy [in] add it to the front or the back
         */
        void addChild(SceneNode::Ptr node, AddPolicy policy=Back);

        /**
         * @brief add a child to the groupnode \b parent
         * @param child [in] the child to add
         * @param parent [in] the parent
         * @param policy [in] add it to the front or the back
         */
        static void addChild(SceneNode::Ptr child, GroupNode::Ptr parent, AddPolicy policy=Back);

        /**
         * @brief test if this group node has the node \b node as child
         * @param node [in] a scene node
         * @return true if \b node is child of this, false otherwise
         */
        bool hasChild(SceneNode::Ptr node);

        /**
         * @brief test if this group node has a child node with name \b nodename
         * @param nodename [in] name of child node
         * @return true if \b nodename is child of this, false otherwise
         */
        bool hasChild(const std::string& nodename);

        /**
         * @brief remove \b node from the children list of this node AND removes this
         * node from the parent list of \b node.
		 *
         * @param node [in] child node that is to be removed
         */
        void removeChild(SceneNode::Ptr node);

        /**
         * @brief remove node with name \b name from the children list of this node AND removes this
         * node from the parent list of \b node.
		 *
		 * @param name [in] child node that is to be removed
         */
        void removeChild(const std::string& name);

        //! transform is relative to parent node
        void setTransform(const rw::math::Transform3D<>& t3d);

        //! @brief get transform
        rw::math::Transform3D<> getTransform();

        //! @brief The transform.
        rw::math::Transform3D<> _t3d;

        //! @brief List of child nodes.
        std::list<SceneNode::Ptr> _childNodes;
    };

}
}

#endif /* GROUPNODE_HPP_ */
