#ifndef RWLIBS_PATHPLANNERS_PRMPLANNER_HPP
#define RWLIBS_PATHPLANNERS_PRMPLANNER_HPP

#include <rw/pathplanning/PathPlanner.hpp>

#include <rw/math/WeightedEuclideanMetric.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/common/Timer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include "PartialIndexTable.hpp"


namespace rwlibs {
namespace pathplanners {
/** @addtogroup pathplanners */
/*@{*/

/**
 * @brief Implements a probabilistic roadmap (PRM) planner.
 * 
 * The PRMPlanner is implemented freely after [1], and has a number of options:
 * - Lazy Collision Checking: Using lazy collision checking as in [2], the planner can be used
 * for single as well as multiple queries. 
 * - Nearest Neighbor Search: The algorithm can either use a partial index table [3] or a simple
 * brute force method to do the nearest neighbor search.
 * - Shortest Path Algorithm: Using the Boost Graph Library, both A* and Dijkstra's Algorithm 
 * may be used for finding the shortest path.
 * 
 * As default the algorithm runs with lazy collision checking, brute force neighbor search and with
 * A* for shortest path search.
 * 
 * As metric the PRMPlanner uses a WeightedEuclideanMetric for which it estimates the weights
 * such that it provides a worst-case estimate of the Cartesian motion of the robots given a 
 * change in the configuration.
 * 
 * Example of use
 * \code
 *      PRMPlanner* prm = new PRMPlanner(device, workcell, state, collisionDetector, resolution);
 *      prm->setCollisionCheckingStrategy(PRMPlanner::LAZY);
 *      prm->setNeighSearchStrategy(PRMPlanner::BRUTE_FORCE);
 *      prm->setShortestPathSearchStrategy(PRMPlanner::A_STAR);
 *      prm->buildRoadmap(1000);
 *      Path path;
 *      bool pathFound = prm->query(qstart, qgoal, path, maxtime);      
 * \endcode  
 * 
 * [1]: Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces, 
 *      L.E. Kavraki, P. Svestka, J-C. Latombe, M.H. Overmars. IEEE Transactions on Robotics
 *      and Automation, Vol. 12, page 566-580, 1996
 * 
 * [2]: Path Planning using Lazy PRM, R. Bohlin, L.E. Kavraki. Proceedings of the IEEE International 
 *      Conference on Robotics and Automation, Vol. 1, p.521-528, 2000
 * 
 * [3]: On Delaying Collision Checking in PRM Planning - Application to Multi-Robot Coordination,
 *      G. Sanchez, J.C. Latombe. The International Journal of Robotics Research, Vol. 21, No. 1, 5-26, 2002
 * 
 */
class PRMPlanner: public rw::pathplanning::PathPlanner
{
public:
    
    /**
     * @brief Constructs PRMPlanner
     * 
     * Constructs a PRMPlanner with a given setup. This method does NOT build the roadmap.
     * The method estimates the movements of the robot to construct a weighted metric as explained
     * in the general introduction. 
     * 
     * @param device [in] Device to plan for
     * @param workcell [in] WorkCell to plan in
     * @param state [in] State giving the setup of the workcell
     * @param collisionDetector [in] CollisionDetector to use
     * @param resolution [in] Cartesian distance the robot is allowed to move between collision checks.  
     */
	PRMPlanner(rw::models::Device* device,
	           rw::models::WorkCell* workcell,
               const rw::kinematics::State& state,
               rw::proximity::CollisionDetector* collisionDetector,
               double resolution);
	
	/**
	 * @brief Destructor
	 */
	virtual ~PRMPlanner();
	
	/**
	 * @brief Build the roadmap with the setup specified
	 * @param nodecount [in] Number of nodes to insert
	 */
	void buildRoadmap(size_t nodecount);
	
	/**
	 * @copydoc rw::pathplanning::PathPlanner::query
	 */
    bool query(const rw::math::Q& qInit,
               const rw::math::Q& qGoal,
               rw::pathplanning::Path& path,
               double timeS);
    
    /**
     * @brief Enumeration for selecting the node neighbor search strategy
     */
    enum NeighborSearchStrategy {
                                    BRUTE_FORCE = 0, /*!< Run through all node and look a which a sufficient close. */ 
                                    PARTIAL_INDEX_TABLE /*!< Use a partial index table to make an more efficient lookup. */  
                                 };
    
    /**
     * @brief Sets up the nearest neighbor search strategy
     * 
     * @param neighborSearchStrategy [in] The nearest neighbor search strategy
     */
    void setNeighSearchStrategy(NeighborSearchStrategy neighborSearchStrategy);
    
    /**
     * @brief Sets up the number of dimensions for the partial index table
     * 
     * This setting only applies when using the PARTIAL_INDEX_TABLE strategy for nearest 
     * neighbor search.
     * 
     * \b dimensions should be within \f$[1; _device->getDOF()]\f$. The optimal value of
     * \dimensions is a tradeoff between memory usage and time. Selecting a value too high compared
     * to the number of nodes in the roadmap may introduce an increase in time due to additional 
     * bookkeeping.
     * 
     * The default value is set to 4, which is found suitable for most devices with 6 or 7 degrees of freedom.
     * 
     * @param dimensions [in] Number of dimensions, which should be
     */
    void setPartialIndexTableDimensions(size_t dimensions);
    
    /**
     * @brief Enumeration for selecting the collision checking strategy 
     */ 
    enum CollisionCheckingStrategy {
                                       LAZY = 0, /*!< Perform a Lazy collision checking (no checking on construction).*/
                                       NODECHECK, /*!< Only check node on construction, leave edges unchecked. */  
                                       FULL /*!<Perform a full check of both nodes and edges. */ 
                                    };

    /**
     * @brief Sets up the collision checking strategy
     * 
     * Note: Do not call this after the buildRoadmap as it may result in paths with collisions
     * @param collisionCheckingStrategy [in] The collision checking strategy  
     */
    void setCollisionCheckingStrategy(CollisionCheckingStrategy collisionCheckingStrategy);
    
    /**
     * @brief Enumeration for selecing the shortest path search strategy
     */
    enum ShortestPathSearchStrategy { 
                                        A_STAR = 0, /*!< Use A* to search for shortest path. */
                                        DIJKSTRA /*!< Use Dijkstra to search for shortest path. */    
                                     };
                                    
    /**
     * @brief Sets up the shortest path search strategy
     * 
     * Generally A* is the fastest algorithm, but given a small roadmap Dijkstra may 
     * perform better.
     *  
     * @param shortestPathSearchStrategy [in] shortestPathSearchStrategy
     */                                    
    void setShortestPathSearchStrategy(ShortestPathSearchStrategy shortestPathSearchStrategy); 
    
    
    void test(size_t i);
private:
    rw::models::Device* _device;
    rw::models::WorkCell* _workcell;
    rw::kinematics::State _state;
    rw::proximity::CollisionDetector* _collisionDetector;
    double _resolution;
    
    rw::pathplanning::PlannerUtil _util;
    rw::math::Q _metricWeights;
    std::auto_ptr<rw::math::Metric<double> > _metric;
    
    
    double _Rneighbor;
    size_t _Nneighbor;
    NeighborSearchStrategy _neighborSearchStrategy;
    size_t _partialIndexTableDimensions;
    CollisionCheckingStrategy _collisionCheckingStrategy;
    
    ShortestPathSearchStrategy _shortestPathSearchStrategy;
    
    /**
     * @brief The data contained in the PRM graph node
     */
    struct NodeData{
        //! Joint configuration
        rw::math::Q q;

        //! Has the node been checked for collision
        bool checked;        
    };

    /**
     * @brief The data contained in the PRM graph edge
     */
    struct EdgeData{
        //! The edge-weight (defined as pPath(left, right))
        double weight;

        /**
         * @brief The resolution to which this node has been checked
         * (if resolution < _resolution then the edge is considered checked)
         */
        double resolution;
        
        rw::math::Q q1;
        rw::math::Q q2;
    };
    
    

    
    /**
     * @brief The PRM (Probabilistic RoadMap)
     */
    typedef boost::adjacency_list<
        boost::listS,
        boost::listS,
        boost::undirectedS,
        NodeData,
        EdgeData> PRM;

    //! The roadmap
    PRM _graph;
    
    //! List of seeds for the enhancement step
    std::vector<rw::math::Q> _seeds;
    
    //! A PRM node
    typedef PRM::vertex_descriptor Node;

    //! A PRM edge
    typedef PRM::edge_descriptor Edge;

    
    boost::shared_ptr<prm::PartialIndexTable<Node> > _partialIndexTable;

    
    bool addEdge(const Node& n1, const Node& n2, double dist);
    
    void addEdges(const Node& node);
    
    Node addNode(const rw::math::Q& q, bool checked);
    
    double estimateRneighbor(size_t roadmapsize);

    bool searchForShortestPathDijkstra(const Node& nInit, const Node& nGoal, std::list<Node>& result);
    bool searchForShortestPathAstar(const Node& nInit, const Node& nGoal, std::list<Node>& result);
    bool inCollision(std::list<Node>& path);
    bool enhanceEdgeCheck(Edge& e);
    void removeCollidingNode(const Node node);
    void removeCollidingEdge(const Edge edge);
 
    void enhanceAround(const rw::math::Q& q);
    void enhanceRoadmap();
    
    size_t _enhanceAroundSeedCount;
    size_t _enhanceRandomFromSeedsCnt;
    size_t _enhanceRandomCnt;
    
    
    class EdgeCompare {
    private:
        PRM* _prm;
    public:   
        EdgeCompare(PRM* prm): _prm(prm) {}
        
        bool operator()(const Edge& e1, const Edge& e2) const {         
            //return 1;
            return (*_prm)[e1].resolution > (*_prm)[e2].resolution; 
        }
    };
    
    
    /**
     * @brief The heuristic distance meassure used by the A* shortest path search
     */
    class PathHeuristic : public boost::astar_heuristic<PRM, double>{
    public:
        /**
         * @brief Creates object
         * @param lazy [in] the lazy PRM path planner
         * @param nGoal [in] the goal node
         */
        PathHeuristic(PRM& prm, const rw::math::Metric<>* metric, const Node& nGoal) :
            _prm(prm),                
            _metric(metric),
            _qGoal(prm[nGoal].q)
        {
        }

        /**
         * @brief Calculates distance from n to goal
         * @param n [in] the node n
         * @return the distance
         */
        double operator()(const Node& n){
            return _metric->distance(_qGoal,
                                     _prm[n].q);                    
        }

    private:
        PRM& _prm;
        //! Controlling pathplanner
        const rw::math::Metric<>* _metric;
        //! Goal configuration
        rw::math::Q _qGoal;        
    };
    

    

};

/* @} */
 
} //end namespace pathplanners
} //end namespace rwlibs

#endif //RWLIBS_PATHPLANNERS_PRMPLANNER_HPP
