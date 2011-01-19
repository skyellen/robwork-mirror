/*
 * NodeVisitor.hpp
 *
 *  Created on: 02/12/2010
 *      Author: jimali
 */

#ifndef NODEVISITOR_HPP_
#define NODEVISITOR_HPP_

#include <rw/graphics/DrawableNode.hpp>
#include <boost/foreach.hpp>

typedef boost::function<bool(Node::Ptr& node, Node::Ptr& parent)> NodeVisitor;

struct DrawableVectorVisitor: public NodeVisitor{

    bool operator()(SceneGraph::Node::Ptr& child, SceneGraph::Node::Ptr& parent){
        BOOST_FOREACH(DrawableNode::Ptr& d, child->_leafNodes){
            if(dmap.find(d.get())!=dmap.end()){
                drawables.push_back(d);
                dmap[d.get()] = 1;
            }
        }
        return false;
    }

    std::vector<DrawableNode::Ptr> drawables;
    std::map<DrawableNode*,int> dmap; // to check if the drawable has allready been added
};

struct FindDrawableVisitor: public NodeVisitor{
    FindDrawableVisitor(std::string name, bool findall):_name(name),_findall(findall){}
    bool operator()(SceneGraph::Node::Ptr& child, SceneGraph::Node::Ptr& parent){
        BOOST_FOREACH(DrawableNode::Ptr& d, child->_leafNodes){
            if(d->getName()==_name){
                if(_findall){
                    _dnodes.push_back(d);
                } else {
                    _dnode = d;
                    return true;
                }
            }
        }
        return false;
    }

    DrawableNode::Ptr _dnode;
    std::vector<DrawableNode::Ptr> _dnodes;
    std::string _name;
    bool _findall;
};


#endif /* NODEVISITOR_HPP_ */
