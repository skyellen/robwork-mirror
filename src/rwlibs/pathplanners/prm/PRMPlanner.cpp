#include "PRMPlanner.hpp"

#include <rw/math/Math.hpp>

#include <boost/vector_property_map.hpp>
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

namespace {
    Timer collisionTimer;
    Timer roadmapBuildTimer;
    Timer queryTimer;
    Timer shortestPathTimer;
    Timer neighborTimer;
    Timer enhanceTimer;
    

    void printTimeStats() {
        std::cout<<"Collision Time = "<<collisionTimer.getTime()<<std::endl;
        std::cout<<"Build RoadMap Time = "<<roadmapBuildTimer.getTime()<<std::endl;
        std::cout<<"Query Time = "<<queryTimer.getTime()<<std::endl;
        std::cout<<"Shortest Path Time = "<<shortestPathTimer.getTime()<<std::endl;
        std::cout<<"Neightbor Search Time = "<<neighborTimer.getTime()<<std::endl;
        std::cout<<"Enhance Time = "<<enhanceTimer.getTime()<<std::endl;
    }
}

PRMPlanner::PRMPlanner(rw::models::Device* device,
                       rw::models::WorkCell* workcell,
                       const rw::kinematics::State& state,
                       rw::proximity::CollisionDetector* collisionDetector,
                       double resolution): 
    _device(device),
    _workcell(workcell),
    _state(state),
    _collisionDetector(collisionDetector),
    _resolution(resolution),
    _util(device, state, collisionDetector),
    _metricWeights(_util.estimateMotionWeights(_workcell->findFrame("Tool"), PlannerUtil::WORSTCASE, 1000)),
    _metric(new WeightedEuclideanMetric<double>(_metricWeights.m()))
{
    _Rneighbor = 0.1;
    _Nneighbor = 20;
    _partialIndexTableDimensions = 4;
    _neighborSearchStrategy = BRUTE_FORCE;
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
    
}
PRMPlanner::~PRMPlanner()
{
};

void PRMPlanner::test(size_t i) {
    std::cout<<"Rneighbor = "<<_Rneighbor<<std::endl;
    std::cout<<"Lower = "<<_device->getBounds().first<<std::endl;
    std::cout<<"Upper = "<<_device->getBounds().second<<std::endl;
    std::cout<<"Weights = "<<_metricWeights<<std::endl;
    prm::PartialIndexTable<Node> pit(_device->getBounds(), _metricWeights, _Rneighbor, i);
    
    
}

double PRMPlanner::estimateRneighbor(size_t roadmapsize) {
    //std::priority_queue<double> queue;
    std::priority_queue<double, std::vector<double>, std::greater<double> > queue;
    std::pair<Q, Q> bounds = _device->getBounds();
    Q qcenter = (bounds.first + bounds.second)/2.0;
    
    for (size_t i = 0; i<roadmapsize; i++) {
        double dist = _metric->distance(qcenter, _util.randomConfig());
        queue.push(dist);
    }
    
    for (size_t i = 0; i<std::min(roadmapsize,_Nneighbor); i++) {
        queue.pop();
    }
    return queue.top();   
}

bool PRMPlanner::addEdge(Node n1, Node n2, double dist) {
    Q q1 = _graph[n1].q;
    Q q2 = _graph[n2].q;

    switch (_collisionCheckingStrategy) {
    case LAZY:
    case NODECHECK: {

        EdgeData data = {dist, dist, q1, q2};
        boost::add_edge(n1, n2, data, _graph);
        break;
    }
    case FULL: {
        //TODO Test if it actually is better to use a binary search
        //which will yield more tests, but with greater probability of finding a collision earlier
        if (!_util.inCollision(q1, q2, (int)std::ceil(dist/_resolution))) {
            EdgeData data = {dist, _resolution, q1, q2};
            boost::add_edge(n1, n2, data, _graph);            
        } else {
            _seeds.push_back((_graph[n1].q + _graph[n2].q)/2.0);
            return false;
        }        
        break;
    }
    
    }
    return true;
}

void PRMPlanner::addEdges(Node node) {
    neighborTimer.resume();
    
    size_t bf_nc = 0;
    size_t pit_nc = 0;
    
    switch (_neighborSearchStrategy) {
    case BRUTE_FORCE:
    {
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
        break;
    }
    case PARTIAL_INDEX_TABLE: 
    {
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

        break;
    }
    }
    neighborTimer.pause();
}

PRMPlanner::Node PRMPlanner::addNode(const Q& q, bool checked) {
    NodeData data = {q, checked};
    Node newNode = add_vertex(data, _graph);
    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE) {
        _partialIndexTable->addNode(newNode, q);
    }
    return newNode;
}



void PRMPlanner::buildRoadmap(size_t nodecount) {
    roadmapBuildTimer.resume();
    _Rneighbor = estimateRneighbor(nodecount);

    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE)
        _partialIndexTable = boost::shared_ptr<prm::PartialIndexTable<Node> >(new prm::PartialIndexTable<Node>(_device->getBounds(), _metricWeights, _Rneighbor, _partialIndexTableDimensions));
    
    size_t cnt = 0;
    while (cnt<nodecount) {    
        Q q = _util.randomConfig();
        if (_collisionCheckingStrategy != LAZY) {        
            if (_util.inCollision(q)) 
                continue;
        }
        Node node = addNode(q, _collisionCheckingStrategy != LAZY);
        addEdges(node);
        cnt++;

    }
    roadmapBuildTimer.pause();
    printTimeStats();
}


void PRMPlanner::enhanceAround(const Q& q) {
    Q ran(q.size());
    for (size_t cnt = 0; cnt<_enhanceAroundSeedCount; cnt++) {
        for (size_t i = 0; i<q.size(); i++) {
            double stddev = _Rneighbor/(1+_metricWeights(i));
            ran(i) = Math::RanNormalDist(q(i), 2*stddev);            
        }
        ran = _util.clampPosition(ran);
        if (_collisionCheckingStrategy != LAZY) {        
            if (_util.inCollision(ran)) 
                continue;
        }
        Node node = addNode(ran, _collisionCheckingStrategy != LAZY);

        addEdges(node);

    }    
}

void PRMPlanner::enhanceRoadmap() {
    enhanceTimer.resume();
    for (size_t cnt = 0; cnt < std::min(_seeds.size(),_enhanceRandomFromSeedsCnt); cnt++) {
        int index = Math::RanI(0, _seeds.size());            
        enhanceAround(_seeds[index]);
    }
    
    for (size_t cnt = 0; cnt < _enhanceRandomCnt; cnt++) {
        enhanceAround(_util.randomConfig());
    }
    enhanceTimer.pause();
}

/**
 * @copydoc rw::pathplanning::PathPlanner::query
 */
bool PRMPlanner::query(const rw::math::Q& qInit,
                       const rw::math::Q& qGoal,
                       Path& path,
                       double timeS) {
    queryTimer.resume();
    std::cout<<"Query"<<std::endl;
    if (_util.inCollision(qInit)) {
        RW_WARN("Init in collision.");
        queryTimer.pause();
        return false;
    }

    if (_util.inCollision(qGoal)) {
        RW_WARN("Goal in collision.");
        queryTimer.pause();
        return false;
    }

    typedef boost::graph_traits<PRM>::vertex_iterator NodePointer;

    Node nInit = addNode(qInit, true);
    addEdges(nInit);
    Node nGoal = addNode(qGoal, true);
    addEdges(nGoal);

    Timer timer;
    timer.reset();
    
    while (timer.getTime() < timeS) {
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
            enhanceRoadmap();
            continue;
        }
        
        bool inCol = inCollision(nodepath);
        if (!inCol) {     
            std::cout<<"Time to find path = "<<timer.getTime()<<std::endl;
            path.clear();
            for (std::list<Node>::iterator it = nodepath.begin(); it != nodepath.end(); ++it) {
                path.push_back(_graph[*it].q);
            }              
            queryTimer.pause();
            printTimeStats();
            return true;
        } 
    } //end for while (timer.getTime() < timeS)
    queryTimer.pause();
    printTimeStats();
    return false;
}


bool PRMPlanner::enhanceEdgeCheck(Edge& e) {
    double resolution = _graph[e].resolution/2.0;
        
    Q q1 = _graph[e].q1;
    Q q2 = _graph[e].q2;
    Q v = q2-q1;
    double length = _metric->distance(v);
    Q vn = v/length;
    
    double p = resolution;
    while (p < length) {
        Q q = q1+p*vn;
        if (_util.inCollision(q))
            return false;
        p += 2*resolution;        
    }
    _graph[e].resolution = resolution;
    return true;
}



bool PRMPlanner::inCollision(std::list<Node>& path) { 
    if (_collisionCheckingStrategy == FULL)
        return false;
    std::vector<Node> nodes(path.begin(), path.end());
    //Run through all nodes to see if they are tested
    if (_collisionCheckingStrategy == LAZY) {
        for(size_t i=0;i<path.size();i++) {
            //Formula such that we check from the ends
            int index1 = (int)std::floor((double)i/2.0) * ( -(i % 2) + (i+1) % 2) + (i % 2) * (path.size()-1);
            Node n = nodes[index1];
            if(!_graph[n].checked){
                if(_util.inCollision(_graph[n].q)){
                    removeCollidingNode(n);
                    return true;
                } else {
                    _graph[n].checked=true;
                }
            }
        }
    }
    

    
    //std::cout<<"All Nodes checked"<<std::endl;
    std::priority_queue<Edge, std::vector<Edge>, EdgeCompare > edgeQueue(EdgeCompare(&_graph), std::vector<Edge>(0));
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
            removeCollidingEdge(edge);
            _seeds.push_back((_graph[edge].q1 + _graph[edge].q2)/2.0);
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
    shortestPathTimer.resume();
    // Perform index mapping 
    typedef boost::graph_traits<PRM>::vertices_size_type t_size_t;
    std::map<Node, t_size_t> indexMapSource;
    boost::associative_property_map<std::map<Node, size_t> > indexMap(indexMapSource);
    boost::graph_traits<PRM>::vertex_iterator vi, vend;
    t_size_t cnt = 0;

    for(boost::tie(vi,vend) = boost::vertices(_graph); vi != vend; ++vi)
        put(indexMap, *vi, cnt++);

    // Initialize property map
    // TODO: change to use index map 
     std::map<Node, Node> pSource;
     boost::associative_property_map<std::map<Node, Node> > p(pSource);

    //std::vector<PRM::vertex_descriptor> p(num_vertices(_graph));
    std::vector<float> d(num_vertices(_graph));
    try {      
        // call astar named parameter interface
        astar_search(
            _graph,
            nInit,
            PathHeuristic(_graph, _metric.get(), nGoal),
            weight_map(get(&EdgeData::weight, _graph)).
            vertex_index_map(indexMap).
            predecessor_map(p).
            /*predecessor_map(&p[0]).*/
            /*distance_map(&d[0]).*/
            visitor(AStarGoalVisitor<Node>(nGoal, _astarTimeOutTime))
            );
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
        for(Node n = nGoal; ; n = p[n]) {
            result.push_front(n);
            if(p[n] == n)
                break;
        }
        shortestPathTimer.pause();
        return true;
    } catch (const AStarGoalVisitor<Node>::AStarTimeOut&) {

        RW_WARN("AStar Timed Out - Running Dijsktra Instead");
        return searchForShortestPathDijkstra(nInit, nGoal, result);
    }
    shortestPathTimer.pause();
    return false;
}


void PRMPlanner::removeCollidingNode(Node node){
    if (_neighborSearchStrategy == PARTIAL_INDEX_TABLE)
        _partialIndexTable->removeNode(node, _graph[node].q);

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
