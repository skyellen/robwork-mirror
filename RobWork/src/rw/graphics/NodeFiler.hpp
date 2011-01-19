/*
 * NodeFiler.hpp
 *
 *  Created on: 02/12/2010
 *      Author: jimali
 */

#ifndef NODEFILER_HPP_
#define NODEFILER_HPP_

typedef boost::function<bool(const Node::Ptr& node)> NodeFilter;

struct NodeTypeExcludeFilter: public SceneGraph::NodeFilter {
    NodeTypeExcludeFilter(int type):_type(type){};
    bool operator()(const SceneGraph::Node::Ptr& child) const { return child->getType()==_type; }
    int _type;
};

struct StaticFilter: public SceneGraph::NodeFilter {
    StaticFilter(bool retValue):_retvalue(retValue){}
    bool operator()(const SceneGraph::Node::Ptr& child) const { return _retvalue; }
    bool _retvalue;
};


#endif /* NODEFILER_HPP_ */
