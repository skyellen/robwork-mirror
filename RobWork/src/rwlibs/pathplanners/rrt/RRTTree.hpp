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


#ifndef RWLIBS_PATHPLANNERS_RRT_RRTTREE_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTTREE_HPP

/**
   @file RRTTree.hpp
*/

#include "RRTNode.hpp"
#include <vector>
#include <boost/foreach.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Tree data type for RRT based planners.
    */
    template <class X>
    class RRTTree
    {
    public:
        typedef X value_type;
        typedef RRTNode<X> node_type;

        RRTTree(const value_type& value)
        {
            add(value, 0);
        }

        node_type& getRoot() const { return *_nodes[0]; }
        node_type& getLast() const { return *_nodes.back(); }

        void add(const value_type& value, node_type* parent)
        {
            _nodes.push_back(new node_type(value, parent));
        }

        typedef typename std::vector<node_type*>::const_iterator const_iterator;

        std::pair<const_iterator, const_iterator>
        getNodes() const
        { return std::make_pair(_nodes.begin(), _nodes.end()); }

        ~RRTTree()
        {
            BOOST_FOREACH(node_type* node, _nodes) {
                delete node;
            }
        }

        size_t size() const { return _nodes.size(); }

        // If speed is important, you could add a version that retrieves
        // pointers to the values instead.
        static
        void getRootPath(node_type& last, std::vector<value_type>& path)
        {
            node_type* node = &last;
            while (node) {
                path.push_back(node->getValue());
                node = node->getParent();
            }
        }

    private:
        std::vector<node_type*> _nodes;

    private:
        RRTTree(const RRTTree&);
        RRTTree& operator=(const RRTTree&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
