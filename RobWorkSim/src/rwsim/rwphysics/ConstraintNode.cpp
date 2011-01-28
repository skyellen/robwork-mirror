#include "ConstraintNode.hpp"

using namespace rwsim::simulator;

namespace {
    bool isPhysicalNode(ConstraintNode::NodeType type){
        if( type==ConstraintNode::Rigid ||
                type==ConstraintNode::Link ||
                type==ConstraintNode::Part )
            return true;
        if( type==ConstraintNode::Fixed
                || type==ConstraintNode::Scripted )
            return true;
        return false;
    }
}

ConstraintNode::ConstraintNode(NodeType type, int id):
    _type(type),
    _isPhysical( isPhysicalNode(type) ),
    _frame(NULL),
    _id(id)
{}

ConstraintNode::ConstraintNode(ConstraintNode* parent, NodeType type, int id):
    _type(type),
    _isPhysical( isPhysicalNode(type) ),
    _parentNode(parent),
    _frame(NULL),
    _id(id)
{}

void ConstraintNode::setType(NodeType type){
    _type = type;
    _isPhysical = isPhysicalNode(type);
}
