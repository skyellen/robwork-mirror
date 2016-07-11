#include "ContactGraph.hpp"
#include "ContactModelFactory.hpp"
#include "CNodePool.hpp"

#include <stack>
#include <boost/foreach.hpp>

#include "ConstraintEdge.hpp"


using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::math;

using namespace rwsim::simulator;

typedef CNodePairMap< ConstraintEdge* > EdgeMap;
typedef FrameMap< ConstraintNode* > FrameNodeMap;

#define BLACK 0x00
#define WHITE 0x01
#define GREY 0x02
#define BLUE 0x03

#define TOUCHING_DIST 0.005
#define PENETRATION_DIST 0.000001
#define SEPERATION_DIST 0.01

namespace {

	bool hasFramePair( const EdgeMap& map, const CNodePair &nodePair ){
		return map[nodePair] != NULL;
	}

	void removeEdgeFromNodes( ConstraintEdge& edge ){
	    edge.getNodes().first->removeEdge(&edge);
	    edge.getNodes().second->removeEdge(&edge);
	}

}

ContactGraph::ContactGraph(CNodePool* pool,
                           ContactModelFactory &factory):
	_factory(factory),
	_pool(pool),
	_pairToEdge(100,NULL),
	_frameToNode(NULL, 100)

{
    BOOST_FOREACH(ConstraintEdge* edge, pool->getEdges() ){
        if(edge==NULL)
            continue;
        std::cout << "Edge : " << std::endl;
        _pairToEdge[ edge->getNodes() ] = edge;
    }

    BOOST_FOREACH(ConstraintNode *node, pool->getNodes()){
        if(node==NULL)
            continue;

        if(node->getFrame()!=NULL){
            std::cout << "node - " << node->getFrame()->getName() << std::endl;
            _frameToNode[ *node->getFrame() ] = node;
        }
    }
}

ContactGraph::~ContactGraph()
{
}

void ContactGraph::rollBack(rw::kinematics::State &state){
    /*EdgeMap::iterator edgeiter = _pairToEdge.begin();
    for(; edgeiter != _pairToEdge.end(); ++edgeiter ){
        ConstraintEdge &edge = *(*edgeiter).second;
        edge.rollBack();
    }
    */
}

void ContactGraph::saveState(){
    /*EdgeMap::iterator edgeiter = _pairToEdge.begin();
    for(; edgeiter != _pairToEdge.end(); ++edgeiter ){
        ConstraintEdge &edge = *(*edgeiter).second;
        edge.saveState();
    } */
}


void ContactGraph::broadPhase(rw::kinematics::State &state,
                              bool shortCircuit){

	// we paint all edges belonging to oFrames BLUE so that we can identify the
	// edges that don't belong to oBodies
    _oFrames.clear();
    _factory.broadPhaseCalc(state, _oFrames);
	FramePairSet::iterator framePair = _oFrames.begin();
	for(;  framePair!=_oFrames.end() ; ++framePair){
	    ConstraintNode *nodeA = _frameToNode[*(*framePair).first];
	    ConstraintNode *nodeB = _frameToNode[*(*framePair).second];
	    if(nodeA==NULL || nodeB==NULL){
	        continue;
	    }

	    // find all frame pairs in oBodies that does not exist in the graph
	    CNodePair nodePair(nodeA,nodeB);
		ConstraintEdge *edge = _pairToEdge.get( nodePair );
		if( edge != NULL  ){
		    //(*edgeiter).second->saveState();
			edge->setState(ConstraintEdge::PersistentProximity);
			edge->setColor(BLUE);
		} else {
		    //ConstraintEdge *newEdge = _pool->createCEdge(nodePair, ConstraintEdge::Physical);
		    ConstraintEdge *newEdge = _factory.CreateConstraintEdge(nodePair);
			_pairToEdge[ newEdge->getNodes() ] = newEdge;
			newEdge->setState( ConstraintEdge::NewProximity );
			newEdge->saveState( );
			newEdge->setColor( BLUE );
		}
	}

	std::vector<ConstraintEdge*> deleteList;
	// for all edges not belonging to _oFrames
	BOOST_FOREACH(ConstraintEdge *ePtr, _pool->getEdges() ){
	    // ConstraintEdges can be NULL so handle this
	    if(ePtr==NULL)
	        continue;

	    const ConstraintEdge &edge = *ePtr;

		if( edge.getColor()==BLUE ){
		    ePtr->setColor(BLACK); // reset color
			continue;
		}

		if( edge.getType() != ConstraintEdge::Physical ){
		    continue;
		}

		if( edge.wasTouching() ){
		    ePtr->setState(ConstraintEdge::VanishingTouch);
		} else {
		    ePtr->setState(ConstraintEdge::VanishingProximity);
		}

		if( edge.isObsolete() ){
		    _pairToEdge[ edge.getNodes() ] = NULL;
		    deleteList.push_back(ePtr); // TODO: use other fixed data structure to optimize
		}
	}

	BOOST_FOREACH(ConstraintEdge *ePtr, deleteList ){
	    removeEdgeFromNodes( *ePtr );
	    _pool->deleteCEdge( ePtr );
	}

	////////////********** applyLogicalCoherenceTest
//	std::cout << "Apply logical and stuff" << std::endl;
	_fEdges.clear();
	BOOST_FOREACH(ConstraintEdge *ePtr, _pool->getEdges() ){
	    // edge must not be NULL
	    RW_ASSERT(ePtr);

	    const ConstraintEdge &edge = *ePtr;

	    // no nodes of the edge can be NULL
	    const CNodePair &npair = edge.getNodes();
        RW_ASSERT(npair.first);
        RW_ASSERT(npair.second);

	    const ConstraintNode &nodeA = *npair.first;
	    const ConstraintNode &nodeB = *npair.second;


	    if( nodeA.isPassive() || nodeB.isPassive() ){
			continue;
		} else if( nodeA.isTrigger() || nodeB.isTrigger() ){
			continue;
//		}else if( _rules.find(e) ){
//			apply rules
//			continue;
		} else if( edge.isResting() ) {
			if( edge.isTouching() ){
				ePtr->setState(ConstraintEdge::PersistentTouch);
			}
			if( edge.isPenetrating() && shortCircuit ){
				assert(0);
			}
			continue;
		}
		// add e to fEdges
		_fEdges.push_back(ePtr);
	}
}

bool ContactGraph::narrowPhase(rw::kinematics::State &state, bool shortCircuit){
	// Narrowphase and shortcuircuiting
    bool penetrating = false;
	std::vector<ConstraintEdge*>::iterator edgeiter = _fEdges.begin();
	for(; edgeiter != _fEdges.end(); ++edgeiter){
	    ConstraintEdge &edge = **edgeiter;
	    if( edge.getState()==ConstraintEdge::Static )
	        continue;

		_factory.narrowPhaseCalc(edge, state);

		penetrating |= edge.isPenetrating();

        if( penetrating && shortCircuit )
            return true;

		if( edge.wasSeperating() ){
		    if( !edge.isSeperating() ){
                edge.setState(ConstraintEdge::NewTouch);
            }
		} else {
            if( edge.isSeperating() ){
                edge.setState( ConstraintEdge::VanishingTouch );
            } else {
                edge.setState( ConstraintEdge::PersistentTouch );
            }
		}
	}
	return penetrating;
}

void ContactGraph::updateContacts(rw::kinematics::State &state){
	// contact determination
	std::vector<ConstraintEdge*>::iterator edgeiter = _fEdges.begin();
	for(; edgeiter != _fEdges.end(); ++edgeiter){
	    ConstraintEdge &edge = **edgeiter;
	    ConstraintNode *nodeA = edge.getNodes().first;
	    ConstraintNode *nodeB = edge.getNodes().second;
		if( nodeA->isPhysical() && nodeB->isPhysical()){
			if( ! edge.isSeperating() ){
				_factory.DetermineContact( edge , state);
			}
		}
	}
}

std::vector< std::vector<ConstraintEdge*> > ContactGraph::computeGroups(){
	std::vector<ConstraintEdge*>::iterator edge = _fEdges.begin();
	for(; edge != _fEdges.end(); ++edge){
		(*edge)->setColor( WHITE );
	}
	std::vector< std::vector<ConstraintEdge*> > groups;
	edge = _fEdges.begin();
	for(; edge != _fEdges.end(); ++edge){
		if( (*edge)->getColor()==WHITE ){
			std::vector<ConstraintEdge*> group;
			traverseGroup(**edge,group);

			groups.push_back(group);
		}
	}
	return groups;
}

// this could be optimized with stack/while aproach instead of nested calls
void ContactGraph::traverseGroup(ConstraintEdge &edge,
                                 std::vector<ConstraintEdge*>& group){
	edge.setColor(GREY);
	group.push_back(&edge);

	ConstraintNode *nodeA = edge.getNodes().first;
	if( ! (nodeA->getNodeType()==ConstraintNode::Fixed ||
	       nodeA->getNodeType()==ConstraintNode::Scripted ) ){
		std::list<ConstraintEdge*>::iterator edgeiter = nodeA->getEdges().begin();
		for(;edgeiter!=nodeA->getEdges().end(); ++edgeiter){
			if( (*edgeiter)->getColor()==WHITE )
				traverseGroup(**edgeiter, group);
		}
	}
	ConstraintNode *nodeB = edge.getNodes().second;
	if( ! (nodeB->getNodeType()==ConstraintNode::Fixed ||
	       nodeB->getNodeType()==ConstraintNode::Scripted) ){
		std::list<ConstraintEdge*>::iterator edgeiter = nodeB->getEdges().begin();
		for(;edgeiter!=nodeB->getEdges().end(); ++edgeiter){
			if( (*edgeiter)->getColor()==WHITE )
				traverseGroup(**edgeiter, group);
		}
	}

	edge.setColor(BLACK);
}

void ContactGraph::remove( ConstraintNode* node){
    Frame *frame = (node->getFrame());
	_frameToNode[ *frame ] = NULL;
	_pool->deleteCNode( node );
}

std::vector< ConstraintNode* > ContactGraph::getConnectedNodes(ConstraintNode* n, ConstraintEdge::EdgeType type){
	std::vector<ConstraintNode*> result;
	std::list<ConstraintEdge*> colored;
	std::stack<ConstraintNode*> nstack;
	nstack.push(n);
	while(!nstack.empty()){
		ConstraintNode* node = nstack.top();
		nstack.pop();
		result.push_back(node);
		std::list<ConstraintEdge*>& edges = node->getEdges();
		std::list<ConstraintEdge*>::iterator edgeiter = edges.begin();
		for(;edgeiter!=edges.end();++edgeiter){
			ConstraintEdge *edge = *edgeiter;
			// only go out of edges that are static constraints
			if( edge->getType()!=type )
				continue;
			// also check if we have added this edge before
			if( edge->getColor() != BLACK )
				continue;
			edge->setColor( BLUE );
			ConstraintNode *cnode;
			if(edge->getNodes().first == node)
				cnode = edge->getNodes().second;
			else
				cnode = edge->getNodes().first;
			result.push_back(cnode);
			colored.push_back(edge);
		}
	}
	// lastly reset color of all edges that where colored
	std::list<ConstraintEdge*>::iterator edgeiter = colored.begin();
	for(;edgeiter!=colored.end();++edgeiter){
		(*edgeiter)->setColor(BLACK);
	}
	return result;
}

std::vector< ConstraintNode* > ContactGraph::getStaticConnectedNodes(ConstraintNode* n){
	return getConnectedNodes(n, ConstraintEdge::Structural);
}

#include <fstream>

namespace {

    void printNodeName(std::ofstream &out, ConstraintNode &node){
        out << "\"";
        std::cout << "GetFrame" <<std::endl;
        if( node.getFrame() != NULL )
             out << node.getFrame()->getName() << " ";
        std::cout << "switch node type." << std::endl;
        switch( node.getNodeType() ){
        case(ConstraintNode::Fixed):
            out << "F" ; break;
        case(ConstraintNode::Link):
            out << "L"; break;
        case(ConstraintNode::CompositeBody):
            out << "C"; break;
        case(ConstraintNode::MultiBody):
            out << "M"; break;
        case(ConstraintNode::Rigid):
            out << "R"; break;
        case(ConstraintNode::Scripted):
            out << "S"; break;
        default:
            out << "D"; break;
            break;
        }
        std::cout << "End switch" << std::endl;
        out << node._idx <<"\" "; // TODO: need unique id
    }

    void printEdge(std::ofstream &out, ConstraintEdge &edge){
        out << "{ edge [color=\"";
        std::cout << "GetType" << std::endl;
        switch(edge.getType()){
        case(ConstraintEdge::Structural):
            out << "green\" ";
            break;
        case(ConstraintEdge::Logical):
            out << "blue\" ";
            break;
        case(ConstraintEdge::Physical):
            out << "red\" ";
            break;
        default:
            out << "black\" ";
            break;
        }

        switch(edge.getState()){
        case(ConstraintEdge::NewProximity):
            out << "label=\"NewProx\" ]"<< std::endl;
            break;
        case(ConstraintEdge::PersistentProximity):
            out << "label=\"PerProx\" ]"<< std::endl;
            break;
        case(ConstraintEdge::VanishingProximity):
            out << "label=\"VanProx\" ]" << std::endl;
            break;
        case(ConstraintEdge::NewTouch):
            out << "label=\"NewTouch\" ]"<< std::endl;
            break;
        case(ConstraintEdge::PersistentTouch):
            out << "label=\"PerTouch\" ]"<< std::endl;
            break;
        case(ConstraintEdge::VanishingTouch):
            out << "label=\"VanTouch\" ]" << std::endl;
            break;
        default:
            out << "]" << std::endl;
            break;
        }
        out << "\t";
        std::cout << "Get node A" << std::endl;
        ConstraintNode *nodeA = edge.getNodes().first;
        if( nodeA == NULL)
            std::cout << "Node is NULL" << std::endl;
        std::cout << "Print node a" << std::endl;
        printNodeName(out, *nodeA);

        out << " -- ";
        std::cout << "Get node B" << std::endl;
        ConstraintNode *nodeB = edge.getNodes().second;
        printNodeName(out, *nodeB);
        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    void printEdges(std::ofstream &out, std::list<ConstraintEdge*> &edges){
        std::cout << "Printting edges!" << std::endl;
        std::list<ConstraintEdge*>::iterator edge = edges.begin();
        for(; edge!=edges.end(); ++edge) {
            ConstraintEdge *e = *edge;
            if(e!=NULL){
                printEdge( out, *e );
            } else
                std::cout << "null ";
        }
    }

    void printNode(std::ofstream &out, ConstraintNode &node){
        std::cout << "Printting nodes!" << std::endl;
        out << "\t";
        printNodeName(out,node);
        switch( node.getNodeType() ){
        case(ConstraintNode::Fixed):
            out << "[ color=green ]; "; break;
        case(ConstraintNode::Link):
            out << "[ color=red ]; "; break;
        case(ConstraintNode::CompositeBody):
            out << "[ color=green ]; "; break;
        case(ConstraintNode::MultiBody):
            out << "[ color=green ]; "; break;
        case(ConstraintNode::Part):
            out << "[ color=lightblue ]; "; break;
        case(ConstraintNode::Rigid):
            out << "[ color=blue ]; "; break;
        case(ConstraintNode::Scripted):
            out << "[ color=yellow ]; "; break;
        default:
            break;
        }
    }

}

void ContactGraph::writeToFile(std::string filename){
    std::ofstream f( filename.c_str() );

    f << "graph \"ContactGraph\" { " << std::endl;
    BOOST_FOREACH(ConstraintNode* node, _pool->getNodes()){
        printNode(f, *node);
        f << std::endl;
    }

    BOOST_FOREACH(ConstraintEdge *edge, _pool->getEdges()){
        printEdge( f, *edge );
    }

    f << " } " << std::endl;
}
