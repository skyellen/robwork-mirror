/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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
#include "LazyPRMPathPlanner.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <rw/models/WorkCell.hpp>

#include <time.h>
#include <float.h>

#include <boost/vector_property_map.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>

using namespace boost;

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

namespace
{
    double norm_inf(const Q& q)
    {
        return norm_inf(q.m());
    }
}

LazyPRMPathPlanner::LazyPRMPathPlanner(
    WorkCell* workcell,
    Device* device,
    CollisionDetector* detector,
    double resolution)
    :
    //    PathPlanner(workcell, detector),
    _utils(device, workcell->getDefaultState(), detector),
    _device(device),
    _localplanner(device, workcell->getDefaultState(), detector, resolution),
    _resolution(resolution),
    _pCollWeights(NULL),
    _pPathWeights(NULL)
{
    _properties.addProperty("Ninit","Initial Node Count", 1000);
    _properties.addProperty(
        "Nenh", "Number of node to add in an enhancement step", 100);
    _properties.addProperty(
        "Mneighb", "Average number of neighbors", 50);

    boost::property_map<PRM, boost::vertex_index_t>::type
        index = get(boost::vertex_index, _graph);

}

LazyPRMPathPlanner::~LazyPRMPathPlanner(){
    if(_pCollWeights)
        delete _pCollWeights;

    if(_pPathWeights)
        delete _pPathWeights;
}

/**
 * \TODO this is not the correct formula, find a bether one!
 */
double calcRneighbour(double E, double V){
    E/=2;
    return (12.2778*(0.07909*V+0.403604*sqrt((E*V)+(0.000586133*V*V))))/V;
}

/*double calcRneighbour(double E,double V){
  return (4.9613*(0.083775*V+0.634917*sqrt((E*V)+(0.00182383*V*V))))/V;
  }*/

void LazyPRMPathPlanner::initialize(Device* device)
{
    _device = device;

    Ninit = _properties.getValue<int>("Ninit");
    Nenh = _properties.getValue<int>("Nenh");
    Mneighb = _properties.getValue<int>("Mneighb");

    std::cout<<"Ninit = "<<Ninit<<std::endl;
    std::cout<<"Nenh = "<<Nenh<<std::endl;
    std::cout<<"Mneigh = "<<Mneighb<<std::endl;

    const size_t dim = _device->getDOF();

    /* Clear roadmap */
    _graph.clear();

    _localplanner.clear();

    /* Initialize m_pCollWeights */
    if(_pCollWeights)
        delete _pCollWeights;

    _pCollWeights = new Q(dim);

    // TODO: find a bether metric!
    for(size_t i = 0; i<dim; i++){
        (*_pCollWeights)[i] = 1.0;
    }

    // Normalize pCollWeights such that the minimum weight is 1.0
    double min = DBL_MAX;
    for(size_t i = 0; i<_pCollWeights->size(); i++){
        if(((*_pCollWeights)[i] < min) && (*_pCollWeights)[i] != 0.0)
            min = (*_pCollWeights)[i];
    }
    *_pCollWeights/= min;

    /* Initialize m_pPathWeights */
    if(_pPathWeights)
        delete _pPathWeights;

    _pPathWeights = new Q(dim);

    Q j1(dim), j2(dim);
    for(size_t i = 0; i < dim; i++){
        j1[i] = 1.0;
        j2[i] = 0.0;
    }

    // TODO
    //*_pPathWeights = (_scene->robot().unNormalize(j1) - _scene->robot().unNormalize(j2));
    *_pPathWeights = j1 - j2;

    /* Calculate Rneighb */
    Rneighb = calcRneighbour((double)Mneighb,(double)Ninit);

    for(size_t i = 0; i < Ninit; i++){
        addNode(_utils.randomConfig(), true);
    }

    _initialized = true;
}

bool LazyPRMPathPlanner::query(
    const Q& qInit,
    const Q& qGoal,
    Path& path,
    double timeS)
{
    RW_ASSERT(_utils.inCollision(qInit)==false);
    RW_ASSERT(_utils.inCollision(qGoal)==false);
    RW_ASSERT(_device != NULL);

    if (_utils.inCollision(qInit)) {
        RW_WARN("Init in collision.");
        return false;
    }

    if (_utils.inCollision(qGoal)) {
        RW_WARN("Goal in collision.");
        return false;
    }

    typedef boost::graph_traits<PRM>::vertex_iterator NodePointer;

    Node nInit = addNode(qInit, false);
    Node nGoal = addNode(qGoal, false);

    RW_ASSERT(norm_inf(_graph[nInit].q - qInit)==0.0);
    RW_ASSERT(norm_inf(_graph[nGoal].q - qGoal)==0.0);

    std::list<Node> shortPath;

    // Main loop
    while(true){
        shortPath.clear();

        bool pathFound = searchForShortestPath(nInit, nGoal, shortPath);


        RW_ASSERT(norm_inf(_graph[nInit].q - qInit)==0.0);
        RW_ASSERT(norm_inf(_graph[nGoal].q - qGoal)==0.0);


        if(!pathFound){
            std::cout<<"Enhance "<<std::endl;
            doNodeEnhancement();
        }else{
            RW_ASSERT(nInit == shortPath.front());
            RW_ASSERT(nGoal == shortPath.back());

            bool success = checkPathForCollision(shortPath);

            // If no collision - assemble path and return true
            if(success){
                RW_ASSERT(nInit == shortPath.front());
                RW_ASSERT(nGoal == shortPath.back());

                RW_ASSERT(norm_inf(_graph[nInit].q - qInit)==0.0);
                RW_ASSERT(norm_inf(_graph[nGoal].q - qGoal)==0.0);

                std::list<Node>::const_iterator it = shortPath.begin();
                while(it != shortPath.end()){
                    path.push_back(_graph[*it].q);
                    it++;
                }

                return true;
            }
        }
    }
    return false; // this is just to keep MSVC6 from complaining. This control path is never reached
}

/**
 * \brief An astar visitor that breaks out of the astar search by
 * throwing an exception when the goal configuration is reached
 * (as recommended in the boost::graph documentation and done in
 * http://www.boost.org/libs/graph/example/astar-cities.cpp)
 */
template<class Node>
class astarGoalVisitor : public boost::default_astar_visitor
{
public:
    /**
     * \brief This is the exception that gets thrown when the
     * goal is reached
     */
    struct FoundGoal{};

    /**
     * \brief Creates object
     * \param nGoal [in] the goal vertex
     */
    astarGoalVisitor(Node nGoal) : _nGoal(nGoal) {}

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
    }
private:
    Node _nGoal;
};

bool LazyPRMPathPlanner::searchForShortestPath(Node nInit, Node nGoal, std::list<Node>& path){

    typedef boost::graph_traits<PRM>::vertices_size_type t_size_t;

    /* Perform index mapping */
    std::map<Node, t_size_t> indexMapSource;
    boost::associative_property_map<std::map<Node, size_t> > indexMap(indexMapSource);
    boost::graph_traits<PRM>::vertex_iterator vi, vend;
    t_size_t cnt = 0;

    for(boost::tie(vi,vend) = boost::vertices(_graph); vi != vend; ++vi)
        put(indexMap, *vi, cnt++);

    /* Initialize property map
     * TODO: change to use index map */
    std::map<Node, Node> pSource;
    boost::associative_property_map<std::map<Node, Node> > p(pSource);

    try {
        astar_search(
            _graph,
            nInit,
            pPathHeuristic(this, nGoal),
            weight_map(get(&EdgeData::weight, _graph)).
            vertex_index_map(indexMap).
            predecessor_map(p).
            /*predecessor_map(&p[0]).*/
            /*distance_map(&d[0]).*/
            visitor(astarGoalVisitor<Node>(nGoal))
            );

    } catch(const astarGoalVisitor<Node>::FoundGoal&) { // found a path to the goal

        for(Node n = nGoal;;
            n = p[n]) {
            path.push_front(n);
            if(p[n] == n)
                break;
        }

        return true;
    }

    return false;
}


bool LazyPRMPathPlanner::checkPathForCollision(const std::list<Node>& path){

    std::vector<Node> nodes(path.begin(), path.end());

    for(int i=0;i<(int)path.size();i++){
        int index =
            i/2* ( -i % 2 + (i+1) % 2) + (i % 2) * (path.size()-1);

        Node n = nodes[index];

        if(!_graph[n].checked){
            if(_utils.inCollision(_graph[n].q)){
                if(_graph[n].random){
                    /* TODO:
                     *
                     * Check all edges from currentNode
                     * if other end is random and not in collision then
                     * add seed
                     */
                }

                removeCollidingNode(n);
                return false;
            }else{
                _graph[n].checked=true;
            }
        }
    }

    for(int i=0;i<(int)path.size()-1;i++){
        int index1 =
            i/2* ( -i % 2 + (i+1) % 2) + (i % 2) * (path.size()-1);
        int index2 =
            index1 + ( -i % 2 + (i+1) % 2);

        Node n1 = nodes[index1];
        Node n2 = nodes[index2];

        Edge e = edge(n1, n2, _graph).first;

        if(_graph[e].resolution != 0.0){
            std::list<Q> dummy1;
            if(_localplanner.query(_graph[n1].q, _graph[n2].q, dummy1)){
                _graph[e].resolution = 0.0;
            }else{
                if(_graph[n1].random && _graph[n2].random){
                    /**
                     * TODO: select seeds
                     */
                }

                removeCollidingEdge(e);
                return false;
            }
        }

    }

    return true;
}


void LazyPRMPathPlanner::removeCollidingNode(const Node node){
    clear_vertex(node, _graph);
    remove_vertex(node, _graph);
}


void LazyPRMPathPlanner::removeCollidingEdge(const Edge edge){
    remove_edge(edge, _graph);
}



void LazyPRMPathPlanner::doNodeEnhancement(){

    // Calculate new Rneighb
    Rneighb = calcRneighbour(Mneighb, num_vertices(_graph));

    /*
     * Add Nenh/2 nodes (atleast 1)
     * to ensure probablistic completeness
     */
    for(size_t i = 0; (i<Nenh/2 || i<1); i++){
        addNode(_utils.randomConfig(), true);
    }

    /*
     * Add Nenh/2 nodes distributed around seeds
     */
    for(size_t i = 0; i<Nenh/2; i++){
        enhanceNode();
    }
}

/**
 * TODO: check if this is correct
 */
void LazyPRMPathPlanner::enhanceNode(){
    if(_seeds.size()==0)
        return;

    double lambda = 1.0;

    Q newNode(_device->getDOF());

    size_t index = rand() % _seeds.size();

    do{

        for(size_t i = 0;i<_seeds[index].size();i++){

            double rand_val = (double)(rand() / (RAND_MAX/2)) - 1.0;

            if((*_pCollWeights)[i] != 0.0)
                newNode[i] = _seeds[index][i] + rand_val * ((lambda * Rneighb)/((*_pCollWeights)[i]));
            else
                newNode[i] = 0.0;

        }

    }while(fabs(norm_inf(newNode))>1.0);

    addNode(newNode, false);
}

double LazyPRMPathPlanner::pColl(const Q& a, const Q &b) const{
    double sum = 0;

    for(size_t i = 0; i < (*_pCollWeights).size(); i++){
        sum += (*_pCollWeights)[i]*(*_pCollWeights)[i]*(a[i]-b[i])*(a[i]-b[i]);
    }

    return sqrt(sum);
}

double LazyPRMPathPlanner::pPath(const Q& a, const Q &b) const{
    double sum = 0;

    for(size_t i = 0; i < (*_pPathWeights).size(); i++){
        sum += (*_pPathWeights)[i]*(*_pPathWeights)[i]*(a[i]-b[i])*(a[i]-b[i]);
    }

    return sqrt(sum);

}

LazyPRMPathPlanner::Node LazyPRMPathPlanner::addNode(const Q& q, bool random){
    NodeData data = {q, random, false};
    Node newNode = add_vertex(data, _graph);
    //_graph[newNode] = data;

    boost::graph_traits<PRM>::vertex_iterator vi, vend;
    for(vi = vertices(_graph).first; *vi != newNode; ++vi){
        double dist = pColl(
            _graph[newNode].q,
            _graph[*vi].q
            );

        if( dist < Rneighb){
            EdgeData data = {
                pPath(_graph[newNode].q, _graph[*vi].q),
                DBL_MAX};
            add_edge(newNode, *vi, data, _graph);
        }
    }
    return newNode;
}

