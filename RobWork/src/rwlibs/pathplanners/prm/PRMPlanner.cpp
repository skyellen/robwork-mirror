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


#include "PRMPlanner.hpp"

#include <limits.h>

#include <rw/math/Math.hpp>
#include <rw/math/Metric.hpp>
#include <rw/math/MetricFactory.hpp>

#include <boost/tuple/tuple.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <queue>

#include "PartialIndexTable.hpp"

using namespace rwlibs::pathplanners;
using namespace rw::math;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;


PRMPlanner::PRMPlanner(rw::pathplanning::QConstraint::Ptr constraint,
	rw::pathplanning::QSampler::Ptr sampler,
    double resolution,
    const rw::models::Device& device,
    const rw::kinematics::State& state)
    :
    _constraint(constraint),
    _sampler(sampler),
    _resolution(resolution),
    _edge(
        QEdgeConstraint::make(
			_constraint,
            MetricFactory::makeEuclidean<Q>(),
            resolution))
{
    initialize(device, state);
}

PRMPlanner::PRMPlanner(
    rw::models::Device* device,
    const rw::kinematics::State& state,
    rw::proximity::CollisionDetector* detector,
    double resolution)
    :
    _constraint(QConstraint::make(detector, device, state)),
    _sampler(QSampler::makeUniform(*device)),
    _resolution(resolution),
    _edge(
        QEdgeConstraint::make(
            _constraint, MetricFactory::makeEuclidean<Q>(), resolution))
{
    initialize(*device, state);
}

void PRMPlanner::initialize(
    const Device& device,
    const State& state)
{
    _bounds = device.getBounds();

    _metricWeights =
        PlannerUtil::estimateMotionWeights(
            device,
            device.getEnd(),
            state,
            PlannerUtil::WORSTCASE,
            1000);


    _metric = MetricFactory::makeWeightedEuclidean<Q>(_metricWeights);

    _Rneighbor = 0.1;
    _Nneighbor = 20;
    _partialIndexTableDimensions = 4;
    _neighborSearchStrategy = PARTIAL_INDEX_TABLE;
    _collisionCheckingStrategy = LAZY;
    _shortestPathSearchStrategy = A_STAR;
    _astarTimeOutTime = 1.0;
    _enhanceAroundSeedCount = 3;
    _enhanceRandomFromSeedsCnt = 20;
    _enhanceRandomCnt = 20;

    collisionTimer.reset();
    collisionTimer.pause();

    roadmapBuildTimer.reset();
    roadmapBuildTimer.pause();

    queryTimer.reset();
    queryTimer.pause();

    shortestPathTimer.reset();
    shortestPathTimer.pause();

    neighborTimer.reset();
    neighborTimer.pause();

    enhanceTimer.reset();
    enhanceTimer.pause();
    _roadmap_initialized = false;
}

PRMPlanner::~PRMPlanner()
{}

/*
void PRMPlanner::test(size_t i) {
    std::cout<<"Rneighbor = "<<_Rneighbor<<std::endl;
    std::cout<<"Lower = "<<_bounds.first<<std::endl;
    std::cout<<"Upper = "<<_bounds.second<<std::endl;
    std::cout<<"Weights = "<<_metricWeights<<std::endl;
    prm::PartialIndexTable<Node> pit(_bounds, _metricWeights, _Rneighbor, i);

}*/

void PRMPlanner::printTimeStats()
{
    std::cout<<"Build RoadMap Time    = "<<roadmapBuildTimer.getTime()<<std::endl;
    std::cout<<"Collision Time        = "<<collisionTimer.getTime()<<std::endl;
    std::cout<<"Query Time            = "<<queryTimer.getTime()<<std::endl;
    std::cout<<"Shortest Path Time    = "<<shortestPathTimer.getTime()<<std::endl;
    std::cout<<"Neightbor Search Time = "<<neighborTimer.getTime()<<std::endl;
    std::cout<<"Enhance Time          = "<<enhanceTimer.getTime()<<std::endl;
}


void PRMPlanner::setNeighborCount(size_t n) {
    _Nneighbor = n;
}

double PRMPlanner::estimateRneighbor(size_t roadmapsize)
{
    std::priority_queue<double, std::vector<double>, std::greater<double> > queue;
    Q qcenter = (_bounds.first + _bounds.second)/2.0;

    for (size_t i = 0; i<roadmapsize; i++) {
        double dist = _metric->distance(qcenter, _sampler->sample());
        queue.push(dist);
    }

    for (size_t i = 0; i<std::min(roadmapsize, _Nneighbor); i++) {
        queue.pop();
    }
	if (queue.empty() )
		RW_THROW("Unable to estimate distance to "<<roadmapsize<<" neighbors");

    return queue.top();
}

bool PRMPlanner::inCollision(const Q& a, const Q& b) const
{
    return
        _constraint->inCollision(a) ||
        _constraint->inCollision(b) ||
        _edge->inCollision(a, b);
}

bool PRMPlanner::addEdge(Node n1, Node n2, double dist)
{
    Q q1 = _graph[n1].q;
    Q q2 = _graph[n2].q;

    switch (_collisionCheckingStrategy) {
    case LAZY: case NODECHECK: {

        EdgeData data = {dist, dist, q1, q2};
        boost::add_edge(n1, n2, data, _graph);
        break;
    }
    case FULL: {
        if (!inCollision(q1, q2)) {
            EdgeData data = { dist, _resolution, q1, q2 };
            boost::add_edge(n1, n2, data, _graph);
        } else {
            _seeds.push_back((_graph[n1].q + _graph[n2].q)/2.0);
            return false;
        }
        break;
    }}

    return true;
}

void PRMPlanner::addEdges(Node node)
{
    neighborTimer.resume();

    size_t bf_nc = 0;
    size_t pit_nc = 0;

    // search for neighbors
    if(_neighborSearchStrategy == BRUTE_FORCE ){
        //Find neighbors using brute force
        boost::graph_traits<PRM>::vertex_iterator vi, vend;
        for(vi = vertices(_graph).first; *vi != node; ++vi) {
            double dist = _metric->distance(_graph[node].q, _graph[*vi].q);

            if( dist < _Rneighbor){
                bf_nc++;
                neighborTimer.pause();
                addEdge(*vi, node, dist);
                neighborTimer.resume();
            }
        }
    } else if(_neighborSearchStrategy == PARTIAL_INDEX_TABLE ){
        std::list<Node> nodes = _partialIndexTable->searchNeighbors(_graph[node].q);

        for(std::list<Node>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            if (*it != node) {
                double dist = _metric->distance(_graph[node].q, _graph[*it].q);
                if( dist < _Rneighbor){
                    pit_nc++;
                    neighborTimer.pause();

                    addEdge(*it, node, dist);

                    neighborTimer.resume();
                }
            }
        }
    } else if(_neighborSearchStrategy == KDTREE ){
        _kdnodesSearchResult.clear();

        Q radi = _metricWeights;
        for(size_t i=0;i<radi.size();i++){
            radi[i] = _Rneighbor/std::max(_metricWeights(i),0.1); // scale the neighbor according to weights
        }
        //std::cout << radi << std::endl;
        // now make vector length _Rneighbor
        double nlen = radi.norm2();
        for(size_t i=0;i<radi.size();i++){
            radi[i] = radi[i]*(_Rneighbor*std::sqrt(_bounds.first.size()))/nlen;
        }
        //std::cout << radi << std::endl;
        //std::cout << radi.norm2() << std::endl;
        //std::cout << (_bounds.second - _bounds.first) << std::endl;

        //std::cout << _metricWeights << std::endl;
        //std::cout << _Rneighbor << std::endl;
        // now normalize the vector
        //double len = radi.norm2();
        //for(int i=0;i<radi.size();i++)
        //    radi[i] = radi[i]/len * _Rneighbor;

        //int divs = (int)std::ceil(diff * weights(i) / r);


        _kdtree->nnSearchElipse(_graph[node].q, radi ,_kdnodesSearchResult);
        //std::cout << "Found: neighN " <<  _kdnodesSearchResult.size() << "\n";

        typedef const rwlibs::algorithms::KDTreeQ<Node>::KDNode* VALUE;
        BOOST_FOREACH(VALUE nnode,  _kdnodesSearchResult ){
            neighborTimer.pause();
            double dist = _metric->distance(_graph[node].q, _graph[nnode->value].q);
            if( dist < _Rneighbor){
                addEdge(nnode->value, node, dist);
            }
            neighborTimer.resume();
        }
    }

    neighborTimer.pause();
}

PRMPlanner::Node PRMPlanner::addNode(const Q& q, bool checked)
{
    NodeData data = {q, checked};
    Node newNode = add_vertex(data, _graph);
    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE) {
        _partialIndexTable->addNode(newNode, q);
    } else if( _neighborSearchStrategy == KDTREE ){
        _kdtree->addNode(q, newNode);
    }
    return newNode;
}

void PRMPlanner::buildRoadmap(size_t nodecount)
{
    roadmapBuildTimer.resume();
    _Rneighbor = estimateRneighbor(nodecount);
    //std::cout<<"Rneighbor = "<<_Rneighbor<<std::endl;

    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE){
        _partialIndexTable = boost::shared_ptr<prm::PartialIndexTable<Node> >(
            new prm::PartialIndexTable<Node>(
                _bounds,
                _metricWeights,
                _Rneighbor,
                (int)_partialIndexTableDimensions));
    } else if( _neighborSearchStrategy == KDTREE ) {
        // TODO: make sure to re-balance tree after construction and once in a while...
        _kdtree = ownedPtr( new rwlibs::algorithms::KDTreeQ<Node>(_bounds.first.size()) );
    }

    // initialize the node map
    size_t cnt = 0;
    while (cnt<nodecount) {
        Q q = _sampler->sample();
        if (_collisionCheckingStrategy != LAZY) {
            if (_constraint->inCollision(q))
                continue;
        }
        Node node = addNode(q, _collisionCheckingStrategy != LAZY);
        addEdges(node);
        cnt++;
		//std::cout<<"cnt = "<<cnt<<std::endl;
    }
    roadmapBuildTimer.pause();
    _roadmap_initialized = true;

    //printTimeStats();

}

void PRMPlanner::enhanceAround(const Q& q)
{
    Q ran(q.size());
    for (size_t cnt = 0; cnt<_enhanceAroundSeedCount; cnt++) {
        for (size_t i = 0; i<q.size(); i++) {
            double stddev = _Rneighbor/(1+_metricWeights(i));
            ran(i) = Math::ranNormalDist(q(i), 2*stddev);
        }
        ran = PlannerUtil::clampPosition(_bounds, ran);
        if (_collisionCheckingStrategy != LAZY) {
            if (_constraint->inCollision(ran))
                continue;
        }
        Node node = addNode(ran, _collisionCheckingStrategy != LAZY);

        addEdges(node);
    }
}

void PRMPlanner::enhanceRoadmap()
{
    enhanceTimer.resume();

    for (size_t cnt = 0; cnt < std::min(
             _seeds.size(),_enhanceRandomFromSeedsCnt); cnt++)
    {
        int index = Math::ranI(0, (int)_seeds.size());
        enhanceAround(_seeds[index]);
    }

    for (size_t cnt = 0; cnt < _enhanceRandomCnt; cnt++)
        enhanceAround(_sampler->sample());

    enhanceTimer.pause();
}

/**
 * @copydoc rw::pathplanning::QToQPlanner::query
 */
bool PRMPlanner::doQuery(
    const rw::math::Q& qInit,
    const rw::math::Q& qGoal,
    QPath& path,
    const StopCriteria& stop)
{
    queryTimer.resume();

    // check that qInit and qGoal are right size
    if(qInit.size() != _bounds.first.size() )
        RW_THROW("qInit input is not the right dimension. " << qInit.size() << "!=" << _bounds.first.size() );
    if(qGoal.size() != _bounds.first.size() )
        RW_THROW("qGoal input is not the right dimension. " << qGoal.size() << "!=" << _bounds.first.size() );
    // now test if input is out of bounds
    for(size_t i=0;i<qInit.size();i++){
        if(qInit[i]<_bounds.first[i] || qInit[i]>_bounds.second[i])
            RW_THROW("qInit input is out of bounds: " << qInit << "<" << _bounds.first.size() );

        if(qGoal[i]<_bounds.first[i] || qGoal[i]>_bounds.second[i])
            RW_THROW("qGoal input is out of bounds: " << qGoal << "<" << _bounds.first.size() );
    }

    if (_constraint->inCollision(qInit)) {
        RW_WARN("Init in collision.");
        queryTimer.pause();
        return false;
    }

    if (_constraint->inCollision(qGoal)) {
        RW_WARN("Goal in collision.");
        queryTimer.pause();
        return false;
    }

    // check if roadmap was initialized, if not then build it
    if( !_roadmap_initialized ){
        RW_WARN("Roadmap was not build/initialized before first query. It will be built now with 1000 nodes!");
        buildRoadmap(1000);
    }

    // test if qInit and qGoal is within bounds of robot
    // typedef boost::graph_traits<PRM>::vertex_iterator NodePointer; //not used

    Node nInit = addNode(qInit, true);
    addEdges(nInit);
    Node nGoal = addNode(qGoal, true);
    addEdges(nGoal);

    Timer timer;
    timer.reset();

    while (!stop.stop()) {
        std::list<Node> nodepath;
        bool pathFound = false;

        switch (_shortestPathSearchStrategy) {
        case A_STAR:
            pathFound = searchForShortestPathAstar(nInit, nGoal, nodepath);
            break;
        case DIJKSTRA:
            pathFound = searchForShortestPathDijkstra(nInit, nGoal, nodepath);
            break;
        }
        if (!pathFound) { //TODO Perhaps make a graph enhancement step
            //std::cout<<"No Path Enhance Roadmap"<<std::endl;
            enhanceRoadmap();
            continue;
        }

        bool inCol = inCollision(nodepath);
        if (!inCol) {
            //std::cout<<"Time to find path = "<<timer.getTime()<<std::endl;
            for (std::list<Node>::iterator it = nodepath.begin(); it != nodepath.end(); ++it) {
                path.push_back(_graph[*it].q);
            }
            queryTimer.pause();
            //printTimeStats();
            return true;
        }
    }

    queryTimer.pause();

    //printTimeStats();
    return false;
}

bool PRMPlanner::enhanceEdgeCheck(Edge& e)
{
    double resolution = _graph[e].resolution/2.0;

    Q q1 = _graph[e].q1;
    Q q2 = _graph[e].q2;
    Q v = q2-q1;
    double length = _metric->distance(v);
    Q vn = v/length;

    double p = resolution;
    while (p < length) {
        Q q = q1+p*vn;
        if (_constraint->inCollision(q))
            return false;
        p += 2*resolution;
    }
    _graph[e].resolution = resolution;
    return true;
}

bool PRMPlanner::inCollision(std::list<Node>& path)
{
    if (_collisionCheckingStrategy == FULL)
        return false;

    std::vector<Node> nodes(path.begin(), path.end());
    //Run through all nodes to see if they are tested
    if (_collisionCheckingStrategy == LAZY) {
        for (int i = 0; i < (int)path.size(); i++) {

            // Formula such that we check from the ends
            int index1 = (int)std::floor((double)i/2.0) * ( -(i % 2) + (i+1) % 2) + (i % 2) * ((int)path.size()-1);

            Node n = nodes[index1];
            if(!_graph[n].checked){
                if(_constraint->inCollision(_graph[n].q)) {
                    removeCollidingNode(n);
                    return true;
                } else {
                    _graph[n].checked=true;
                }
            }
        }
    }

    //std::cout<<"All Nodes checked"<<std::endl;
    std::priority_queue<Edge, std::vector<Edge>, EdgeCompare > edgeQueue(
        EdgeCompare(&_graph), std::vector<Edge>(0));

//    std::priority_queue<Segment, std::vector<Segment>, SegmentCompare > edgeQueue(SegmentCompare(&_graph), std::vector<Segment>(0));
    //Run through all edges
    for(size_t i=0;i<path.size()-1;i++){
        Node n1 = nodes[i];
        Node n2 = nodes[i+1];

        Edge e = edge(n1, n2, _graph).first;

        if (_graph[e].resolution > _resolution) {
            edgeQueue.push(e);
        }
    }
 //   std::cout<<"Queue Constructed"<<std::endl;
    while (!edgeQueue.empty()) {
        Edge edge = edgeQueue.top();
        edgeQueue.pop();

        bool enhanced = enhanceEdgeCheck(edge);
        if (!enhanced) {
            _seeds.push_back((_graph[edge].q1 + _graph[edge].q2)/2.0);
			removeCollidingEdge(edge);
            return true;
        }
        if (_graph[edge].resolution > _resolution)
            edgeQueue.push(edge);

    }
    return false;
}





bool PRMPlanner::searchForShortestPathDijkstra(const Node& nInit, const Node& nGoal, std::list<Node>& result) {
    shortestPathTimer.resume();
    typedef boost::graph_traits<PRM>::vertices_size_type t_size_t;

    std::map<Node, t_size_t> indexMapSource;
    boost::associative_property_map<std::map<Node, size_t> > indexMap(indexMapSource);
    boost::graph_traits<PRM>::vertex_iterator vi, vend;
    t_size_t cnt = 0;

    for(boost::tie(vi,vend) = boost::vertices(_graph); vi != vend; ++vi)
        put(indexMap, *vi, cnt++);


    std::map<Node, Node> pSource;
    boost::associative_property_map<std::map<Node, Node> > p(pSource);
    boost::dijkstra_shortest_paths(_graph,
                                   nInit,
                                   boost::predecessor_map(p).
                                   vertex_index_map(indexMap).
                                   weight_map(get(&EdgeData::weight, _graph)));

    Node n = nGoal;
    for(; ; n = p[n]) {
        result.push_front(n);
        if(p[n] == n)
            break;
    }
    shortestPathTimer.pause();
    if (n == nInit)
        return true;
    else
        return false;
}




        /**
         * \brief An astar visitor that breaks out of the astar search by
         * throwing an exception when the goal configuration is reached
         * (as recommended in the boost::graph documentation and done in
         * http://www.boost.org/libs/graph/example/astar-cities.cpp)
         */
        template<class Node>
        class AStarGoalVisitor : public boost::default_astar_visitor
        {
        public:
            /**
             * \brief This is the exception that gets thrown when the
             * goal is reached
             */
            struct FoundGoal {};

            struct AStarTimeOut {};

            /**
             * \brief Creates object
             * \param nGoal [in] the goal vertex
             */
            AStarGoalVisitor(Node nGoal, double timeoutTime):
                _nGoal(nGoal),
                _astarTimeOutTime(timeoutTime)
            {
                _astarFallBackTimer.reset();
                _astarFallBackTimer.resume();
            }

            /**
             * \brief Checks to see if goal is reached, if
             * goal is reached FoundGoal is thrown
             *
             * \param n [in] the node reached
             * \param g [in] the graph in which node n belongs
             * \throws FoundGoal if goal was found
             */
            template <class Graph>
            void examine_vertex(Node n, Graph& g) {
                if(n == _nGoal)
                    throw FoundGoal();

                /*if (_astarFallBackTimer.getTime() > _astarTimeOutTime) {
                    std::cout<<"throws"<<std::endl;
                    throw AStarTimeOut();
                }*/
            }
        private:
            Node _nGoal;
            double _astarTimeOutTime;
            Timer _astarFallBackTimer;
        };


bool PRMPlanner::searchForShortestPathAstar(const Node& nInit, const Node& nGoal, std::list<Node>& result) {
	//std::cout<<"A*"<<std::endl;std::cout.flush();
    shortestPathTimer.resume();
    // Perform index mapping
    typedef boost::graph_traits<PRM>::vertices_size_type t_size_t;
    std::map<Node, t_size_t> indexMapSource;
    boost::associative_property_map<std::map<Node, size_t> > indexMap(indexMapSource);
    boost::graph_traits<PRM>::vertex_iterator vi, vend;
    t_size_t cnt = 0;

	//std::cout<<"A* A"<<std::endl;std::cout.flush();
    for(boost::tie(vi,vend) = boost::vertices(_graph); vi != vend; ++vi)
        put(indexMap, *vi, cnt++);
	//std::cout<<"A* B"<<std::endl;std::cout.flush();
    // Initialize property map
    // TODO: change to use index map
     std::map<Node, Node> pSource;
     boost::associative_property_map<std::map<Node, Node> > p(pSource);

    //std::vector<PRM::vertex_descriptor> p(num_vertices(_graph));
    std::vector<float> d(num_vertices(_graph));
    try {
		//std::cout<<"A* C"<<std::endl;std::cout.flush();
		//const boost::bundle_property_map<PRM, boost::detail::edge_desc_impl<boost::undirected_tag, void*>, rwlibs::pathplanners::PRMPlanner::EdgeData, double>
		//	&pmap = get(&EdgeData::weight, _graph);

		// call astar named parameter interface
		// TODO: PRM Planner fails with gcc 4.4  -  astar_search disabled = not working
        astar_search(
            _graph,
            nInit,
            PathHeuristic(_graph, _metric.get(), nGoal),
            weight_map(get(&EdgeData::weight, _graph)).
            vertex_index_map(indexMap).predecessor_map(p).
            visitor(AStarGoalVisitor<Node>(nGoal, _astarTimeOutTime))
            );

		//std::cout<<"A* D"<<std::endl;std::cout.flush();
        /*astar_search(

            _graph,
            nInit,
            PathHeuristic(_graph, _metric.get(), nGoal),
            weight_map(get(&EdgeData::weight, _graph)).
            vertex_index_map(indexMap).
            predecessor_map(p).
            visitor(AStarGoalVisitor<Node>(nGoal))
            );*/
    } catch (const AStarGoalVisitor<Node>::FoundGoal&) { // found a path to the goal
        //std::cout<<"A* E "<<std::endl;std::cout.flush();
		for(Node n = nGoal; ; n = p[n]) {
            result.push_front(n);
            if(p[n] == n)
                break;
        }
        shortestPathTimer.pause();
		//std::cout<<"A* F "<<result.size()<<std::endl;std::cout.flush();
        return true;
    } catch (const AStarGoalVisitor<Node>::AStarTimeOut&) {
		//std::cout<<"A* G"<<std::endl;std::cout.flush();
        RW_WARN("AStar Timed Out - Running Dijsktra Instead");
        return searchForShortestPathDijkstra(nInit, nGoal, result);
    }
    shortestPathTimer.pause();
    return false;
}


void PRMPlanner::removeCollidingNode(Node node){
    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE){
        _partialIndexTable->removeNode(node, _graph[node].q);
    } else if( _neighborSearchStrategy == KDTREE ) {
        _kdtree->removeNode( _graph[node].q );
    }

    clear_vertex(node, _graph);
    remove_vertex(node, _graph);
}


void PRMPlanner::removeCollidingEdge(Edge edge){
    remove_edge(edge, _graph);
}

void PRMPlanner::setNeighSearchStrategy(NeighborSearchStrategy neighborSearchStrategy) {
    _neighborSearchStrategy = neighborSearchStrategy;
}

void PRMPlanner::setPartialIndexTableDimensions(size_t dimensions) {
    _partialIndexTableDimensions = dimensions;
}

void PRMPlanner::setCollisionCheckingStrategy(CollisionCheckingStrategy collisionCheckingStrategy) {
    _collisionCheckingStrategy = collisionCheckingStrategy;
}

void PRMPlanner::setShortestPathSearchStrategy(ShortestPathSearchStrategy shortestPathSearchStrategy) {
    _shortestPathSearchStrategy = shortestPathSearchStrategy;
}

void PRMPlanner::setAStarTimeOutTime(double timeout) {
    _astarTimeOutTime = timeout;
}
