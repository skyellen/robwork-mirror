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


#ifndef RWLIBS_PATHPLANNERS_RRT_RRTNODE_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTNODE_HPP

/**
   @file RRTNode.hpp
*/

#include <vector>
#include <boost/foreach.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    template <class X>
    class RRTTree;

    /**
       @brief Node type for trees of RRT based planners.
    */
    template <class X>
    class RRTNode
    {
    public:
        typedef RRTNode<X> node_type;
        typedef X value_type;

        node_type* getParent() const { return _parent; }
        const value_type& getValue() const { return _value; }

    private:
        friend class RRTTree<X>;

        RRTNode(const value_type& value, node_type* parent) :
            _value(value),
            _parent(parent)
        {}

        value_type _value;
        node_type* _parent;

        RRTNode(const RRTNode&);
        RRTNode& operator=(const RRTNode&);
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
