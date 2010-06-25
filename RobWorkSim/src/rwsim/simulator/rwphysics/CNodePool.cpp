#include "CNodePool.hpp"

using namespace rwsim::simulator;

CNodePool::CNodePool(int nrNodes, int nrEdges)
{

}

ConstraintNode *CNodePool::createCNode(ConstraintNode::NodeType type){
    if(!_freeNodeIDs.empty()){
        int id = _freeNodeIDs.top();
        _freeNodeIDs.pop();
        _nodes[id]->setDeleted(false);
        _nodes[id]->setID(id);
        _nodes[id]->setType(type);
        return _nodes[id];
    }
    _nodes.push_back( new ConstraintNode(type, _nodes.size()) );
    return _nodes.back();
}

void CNodePool::deleteCNode(ConstraintNode* node){
    node->setDeleted(true);
    _freeNodeIDs.push( node->getID() );
}


ConstraintEdge *CNodePool::createCEdge(const CNodePair& pair, ConstraintEdge::EdgeType type){
    if(!_freeEdgeIDs.empty()){
        int id = _freeEdgeIDs.top();
        _freeEdgeIDs.pop();
        _edges[id]->setDeleted(false);
        _edges[id]->setID(id);
        _edges[id]->setType(type);
        return _edges[id];
    }
    _edges.push_back( new ConstraintEdge(pair, type, _edges.size()) );
    return _edges.back();
}

void CNodePool::deleteCEdge(ConstraintEdge* edge){
    edge->setDeleted(true);
    _freeEdgeIDs.push( edge->getID() );
}

const std::vector<ConstraintNode*>& CNodePool::getNodes() const{
    return _nodes;
}

const std::vector<ConstraintEdge*>& CNodePool::getEdges() const{
    return _edges;
}
