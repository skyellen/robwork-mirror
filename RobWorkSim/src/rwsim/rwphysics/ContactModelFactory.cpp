#include "ContactModelFactory.hpp"

#include "ContactModel.hpp"
#include "CNodePool.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
//#include <rw/proximity/ProximityStrategyFactory.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <boost/foreach.hpp>

using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
using namespace rw::math;
using namespace rw::kinematics;

using namespace rwsim::simulator;
using namespace rwsim::dynamics;

namespace {

    class MyContactInfo
    {

        public:
            MyContactInfo()
            {}

            //DistanceResult narrowCache;
            //MultiDistanceResult contactCache;
            ProximityStrategyData dataCache;
    };


    void deriveContactInfo(Transform3D<>& wTa, Transform3D<>& wTb,
                           Vector3D<>& p1, Vector3D<>& p2,
                           double dist,
                           ContactPoint& contact)
    {
        contact.pA = wTa * p1;
        contact.pB = wTa * p2;
        double len = (contact.pB-contact.pA).norm2();
        contact.n = (contact.pB-contact.pA)/len;
        contact.p = contact.n*(len/2) + contact.pA;
        contact.dist = len;
        contact.tImpulse = 0;
        contact.nImpulse = 0;
    }

/*    Body* getBody(ConstraintNode &node){
    	std::cout << "GetBody" << std::endl;
    	ConstraintNode::NodeType type = node.getNodeType();
	    if( type == ConstraintNode::Rigid ||
	        type == ConstraintNode::Link ||
	        type == ConstraintNode::Fixed){
	    	std::cout << "Rigid/Link/Fixed" << std::endl;
	    	return (Body*) &node;
	    } else if( type == ConstraintNode::Part ){
	    	std::cout << "Part" << std::endl;
	        return (Body*) node.getParentNode();
	    }
	    std::cout << "Unsupported node type!" << std::endl;
    	return NULL;
    }
*/

	void Filter(
				MultiDistanceResult &dists,
				std::list<FilteredPoint>& blobs,
				double sepDist)
	{
		for(size_t i=0; i<dists.p1s.size(); i++){
			const Vector3D<> &v = dists.p1s[i];
			bool inBlob = false;
			BOOST_FOREACH(FilteredPoint& p, blobs){
			//for(int j=0; j<blobs.size(); j++){
				double dist = MetricUtil::dist2<Vector3D<> >(v,p.p);
				if( dist < sepDist ){
					// point is inside blob, break loop
					if( dists.distances[i] < p.dist  ){
						p.p1 = v;
						p.p2 = dists.p2s[i];
						p.dist = dists.distances[i];
					}
					inBlob = true;
					p.hits++;
					break;
				} else if(dist<sepDist*2) {
					// discard point
					inBlob = true;
					break;
				}
			}
			if( !inBlob )
				blobs.push_back( FilteredPoint(v,dists.p2s[i],dists.distances[i]) );
		}
	}

}

ContactModelFactory::ContactModelFactory(
        DynamicWorkCell *dwc,
        CNodePool *pool):
    _dwc(dwc),
    _pool(pool),
    _touchDist(0.01),
    _penDist(0.00001),
    _sepDist(0.1)
{

    _narrowStrategy = new ProximityStrategyPQP();
    CollisionToleranceStrategy *tStrat = _narrowStrategy;
    CollisionStrategy::Ptr cStrat = CollisionStrategy::make(tStrat, _sepDist);
    _toleranceDetector = new CollisionDetector(_dwc->getWorkcell(), cStrat );
}

void ContactModelFactory::broadPhaseCalc(rw::kinematics::State &state, rw::kinematics::FramePairSet &oFrames){
    std::cout << "* Nr of Collisions: " << oFrames.size() << std::endl;
    CollisionDetector::QueryResult res;
    _toleranceDetector->inCollision( state, &res, false);
    oFrames = res.collidingFrames;
    std::cout << "* Nr of Collisions: " << oFrames.size() << std::endl;
}

void ContactModelFactory::narrowPhaseCalc(ConstraintEdge& edge, rw::kinematics::State &state){
    std::cout << "NARROWPHASE CALC" << std::endl;
    // perform distance calculation and find pricipal contact points
    // that is find relevant contact points and save it in edge


    if( edge.data==NULL ){
        std::cout << "ERROR: Edge is not a ContactEdge" << std::endl;
    }
    if( edge.getType()==ConstraintEdge::Physical )
        std::cout << "Edge is physical" << std::endl;
    else
        std::cout << "Edge is not physical" << std::endl;

    Frame *frameA = edge.getNodes().first->getFrame();
    Frame *frameB = edge.getNodes().second->getFrame();
    if( frameA==NULL || frameB==NULL )
        return;

    Transform3D<> wTa = Kinematics::worldTframe(frameA, state);
    Transform3D<> wTb = Kinematics::worldTframe(frameB, state);

    DistanceResult &result = ((DistanceStrategy*)_narrowStrategy)->distance(
                                          frameA,wTa,frameB,wTb,
                                          ((MyContactInfo*)edge.data)->dataCache);

    /*if( !res ){
        std::cout<<"No collision models exist for frame" << std::endl;
        edge.setDistance(100);
        return;
    }*/
    edge.setDistance(result.distance);
    std::cout << "MIN DISTANCE: " << result.distance << std::endl;
}


ConstraintEdge* ContactModelFactory::CreateConstraintEdge(CNodePair& nodes){
    // The type of contact edge is determined from the type of the
    // two nodes

    ConstraintEdge::EdgeType type;
    if( nodes.first->getNodeType()==ConstraintNode::CompositeBody ||
            nodes.second->getNodeType()==ConstraintNode::CompositeBody ||
            nodes.first->getNodeType()==ConstraintNode::MultiBody ||
            nodes.second->getNodeType()==ConstraintNode::MultiBody){
        type = ConstraintEdge::Structural;
    } else if( nodes.first->isPhysical() &&
               nodes.second->isPhysical() ){
        type = ConstraintEdge::Physical;
    } else {
        type = ConstraintEdge::Logical;
    }
    ConstraintEdge *edge;

    // The type of contact model is determined from the type of the
    // two bodies, for now we use a default model.
    std::cout << "Creating edge! for nodes: " << nodes.first->getFrame()->getName() <<  " and "
    										  << nodes.second->getFrame()->getName()<< std::endl;

    if( type == ConstraintEdge::Physical ){
    	std::cout << "PHYSICAL" << std::endl;
    	edge = _pool->createCEdge(nodes, ConstraintEdge::Physical );
    	edge->setThresholds(_touchDist,_penDist, _sepDist);
    	edge->data = (void*)new MyContactInfo();
        //edge = new MyContactEdge(nodes,_touchDist,_penDist, _sepDist);

        RWBody *bodyA = nodes.first->getBody();
        RWBody *bodyB = nodes.second->getBody();

        //ContactModel *model = new ContactModel( *bodyA, *bodyB, this);
        ContactModel *model = new ContactModel( *nodes.first, *nodes.second, this);
        Contact *contact = new Contact( model );
        contact->bodyA = bodyA;
        contact->bodyB = bodyB;

        // TODO: determine rest coeff and other contact related
        // parameters from body info, for now we use default params
        contact->nColRestCoeff = 0.1;
        contact->nConRestCoeff = 0.1;
        contact->staticFriction = 0.4;

        edge->_contact = contact;
    } else {
    	std::cout << "NOT PHYSICAL" << std::endl;
        edge = new ConstraintEdge(nodes, type, _touchDist, _penDist, _sepDist);
    }

    return edge;
}

/**
 * Determines contact information from multiple cooliding points
 */
void ContactModelFactory::DetermineContact(ConstraintEdge &e, rw::kinematics::State& state){
    std::cout << "* Determine contact" << std::endl;
    MyContactInfo &edgeInfo = *((MyContactInfo*)e.data);
    std::cout << " Get frames! " << std::endl;
    Frame *frameA = NULL;// = (Frame*) edgeInfo.narrowCache.f1;
    Frame *frameB = NULL;// = (Frame*) edgeInfo.narrowCache.f2;

    if( frameA==NULL || frameB==NULL )
        std::cout << " Frame is NULLLLLL" << std::endl;

    std::cout << "frames loaded " << std::endl;
    Transform3D<> wTa = Kinematics::worldTframe(frameA, state);
    Transform3D<> wTb = Kinematics::worldTframe(frameB, state);

    std::cout << " calc multi distance result: " << _touchDist << std::endl;

    MultiDistanceResult &result = _narrowStrategy->distances(
                                  frameA,wTa,
                                  frameB,wTb,
                                  _touchDist,
                                  edgeInfo.dataCache);

    if(result.distances.size()==0){
    	_filteredPoints.clear();
        e._contact->contactPoints.resize(0);
        return;
    }

    size_t nrOfContacts = result.distances.size();
    // TODO: filter away points that lie close to each other
    Filter(result, _filteredPoints, 0.02);


    std::cout << "NR OF CONTACTS FOUND: " << nrOfContacts << std::endl;
    // resize the ContactPoint list so all contacts can fit into it
    e._contact->contactPoints.resize(nrOfContacts);

    size_t idx = 0;
    std::list<FilteredPoint>::iterator fiter = _filteredPoints.begin();
    while(fiter!=_filteredPoints.end()){
		FilteredPoint &p = *fiter;
    	if(p.hits==0){
    		fiter = _filteredPoints.erase(fiter);
    	} else {
			ContactPoint &contact = e._contact->contactPoints[idx++];
	        deriveContactInfo(wTa,wTb,p.p1,p.p2,p.dist,contact);
	        p.hits = 0;
	        p.p = p.p1;
	        ++fiter;
    	}
	}
    e._contact->contactPoints.resize(idx);
	std::cout << "NR OF CONTACTS AFTER FILTER: " << idx << std::endl;
/*    for(size_t j=0; j< nrOfContacts; j++){
        ContactPoint &contact = edge._contact->contactPoints[j];
        Vector3D<> &p1 = result.p1s[j];
        Vector3D<> &p2 = result.p2s[j];
        double dist = result.distances[j];
        deriveContactInfo(wTa,wTb,p1,p2,dist,contact);
    }
*/
}

/*
void ContactModelFactory::DetermineContact(ConstraintEdge &edge, rw::kinematics::State& state){
    // This is actually an update of the contact info
    Frame *frameA = (Frame*) edge.getDistResult().f1;
    Frame *frameB = (Frame*) edge.getDistResult().f2;

    Transform3D<> wTa = rw::kinematics::Kinematics::WorldTframe(frameA,state);
    Transform3D<> wTb = rw::kinematics::Kinematics::WorldTframe(frameB,state);

    Vector3D<> p1 = wTa * edge.getDistResult().p1;
    Vector3D<> p2 = wTb * edge.getDistResult().p2;

    if( edge.getState()==ConstraintEdge::NewTouch ){
        // this is first contact, set all contact variables accordingly
        edge._contact.isFirstContact = true;
        edge._contact.p1Init = p1;
        edge._contact.p2Init = p2;
    } else if( edge.getState() == ConstraintEdge::PersistentTouch ){
        // this is not the first contact s�
        edge._contact.isFirstContact = false;
    }
    edge._contact.p1 = p1;
    edge._contact.p2 = p2;
    double len = (p2-p1).norm2();
    edge._contact.n1 = (p2-p1)/len;
    edge._contact.n2 = (p1-p2)/len;
    edge._contact.dist = edge.getDistResult().distance;

    std::cout << "Determining contact info!!" << std::endl;
    edge._contact.print();
}*/
