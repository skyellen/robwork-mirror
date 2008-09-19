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

#ifndef rwlibs_pathplanners_rrt_RRTNode_HPP
#define rwlibs_pathplanners_rrt_RRTNode_HPP

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
