/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_pathplanners_rrt_RRTTree_HPP
#define rwlibs_pathplanners_rrt_RRTTree_HPP

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
