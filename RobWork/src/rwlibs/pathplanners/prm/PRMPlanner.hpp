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


#ifndef RWLIBS_PATHPLANNERS_PRMPLANNER_HPP
#define RWLIBS_PATHPLANNERS_PRMPLANNER_HPP

#include <rw/pathplanning/QToQPlanner.hpp>

#include <rw/math/Metric.hpp>
#include <rw/common/Timer.hpp>

#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


#include "PartialIndexTable.hpp"

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace pathplanning { class QConstraint; } }
namespace rw { namespace pathplanning { class QEdgeConstraint; } }
namespace rw { namespace pathplanning { class QSampler; } }
namespace rw { namespace proximity { class CollisionDetector; } }

namespace rwlibs { namespace pathplanners {
    /** @addtogroup pathplanners */
    /*@{*/

    /**
     * @brief Implements a probabilistic roadmap (PRM) planner.
     *
     * The PRMPlanner is implemented freely after [1], and has a number of options:
     *
     * - Lazy Collision Checking: Using lazy collision checking as in [2], the
     * planner can be used for single as well as multiple queries.
     *
     * - Nearest Neighbor Search: The algorithm can either use a partial index
     * table [3] or a simple brute force method to do the nearest neighbor
     * search.
     *
     * - Shortest Path Algorithm: Using the Boost Graph Library, both A* and
     * Dijkstra's Algorithm may be used for finding the shortest path.
     *
     * As default the algorithm runs with lazy collision checking, brute force
     * neighbor search and with A* for shortest path search.
     *
     * As metric the PRMPlanner uses a WeightedEuclideanMetric for which it
     * estimates the weights such that it provides a worst-case estimate of the
     * Cartesian motion of the robots given a change in the configuration.
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
     * [1]: Probabilistic Roadmaps for Path Planning in High-Dimensional
     *      Configuration Spaces, L.E. Kavraki, P. Svestka, J-C. Latombe, M.H.
     *      Overmars. IEEE Transactions on Robotics and Automation, Vol. 12, pages
     *      566-580, 1996
     *
     * [2]: Path Planning using Lazy PRM, R. Bohlin, L.E. Kavraki. Proceedings
     *      of the IEEE International Conference on Robotics and Automation, Vol. 1,
     *      pages 521-528, 2000
     *
     * [3]: On Delaying Collision Checking in PRM Planning - Application to
     *      Multi-Robot Coordination, G. Sanchez, J.C. Latombe. The International
     *      Journal of Robotics Research, Vol. 21, No. 1, pages 5-26, 2002
     */
    class PRMPlanner: public rw::pathplanning::QToQPlanner
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<PRMPlanner> Ptr;

        /**
         * @brief Constructs PRMPlanner
         *
         * Constructs a PRMPlanner with a given setup. This method does NOT build the roadmap.
         * The method estimates the movements of the robot to construct a weighted metric as explained
         * in the general introduction.
         *
         * @param device [in] Device to plan for
         * @param state [in] State giving the setup of the workcell
         * @param collisionDetector [in] CollisionDetector to use
         *
         * @param resolution [in] Cartesian distance the robot is allowed to move
         * between collision checks.
         */
        PRMPlanner(
            rw::models::Device* device,
            const rw::kinematics::State& state,
            rw::proximity::CollisionDetector* collisionDetector,
            double resolution);

        /**
           @brief Constructs PRMPlanner

           @param constraint [in] Collision constraint
           @param sampler [in] Configuration space sampler
           @param resolution [in] Collision checking resolution
           @param device [in] Device characteristics
           @param state [in] State of rest of the workcell
        */
        PRMPlanner(
        	rw::common::Ptr<rw::pathplanning::QConstraint> constraint,
			rw::common::Ptr<rw::pathplanning::QSampler> sampler,
            double resolution,
            const rw::models::Device& device,
            const rw::kinematics::State& state);

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
         * @brief Sets the desired average number of neighbors. Default value is 20.
         * @param n [in] Desired average number of neighbors
         */
        void setNeighborCount(size_t n);

        /**
         * @brief Enumeration for selecting the node neighbor search strategy
         */
        enum NeighborSearchStrategy {
            BRUTE_FORCE = 0, /*!< Run through all node and look a which a sufficient close. */
            PARTIAL_INDEX_TABLE, /*!< Use a partial index table to make an more efficient lookup. */
            KDTREE /*!< Use KD tree for neighbor search */
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
         * \b dimensions should be within \f$[1; _device->getDOF()]\f$. The optimal
         * value of \b dimensions is a tradeoff between memory usage and time.
         * Selecting a value too high compared to the number of nodes in the roadmap
         * may introduce an increase in time due to additional bookkeeping.
         *
         * The default value is set to 4, which is found suitable for most devices
         * with 6 or 7 degrees of freedom.
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

        /**
         * @brief Sets the max time of A* before terminating and calling dijkstra
         *
         * The A* implementation in the boost graph library has a reported bug, which on
         * some platforms in rare occasions may cause it to loop infinitely. If A* uses
         * more than this specified time it will break off and call dijkstra instead.
         *
         * Default value for this timeout is 1second.
         *
         * @brief timeout [in] Timeout time.
         */
        void setAStarTimeOutTime(double timeout);

        /**
         * @brief print timing stats from previous run.
         */
        void printTimeStats();

    private:
        bool doQuery(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            rw::trajectory::QPath& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        bool _roadmap_initialized;
        rw::common::Ptr<rw::pathplanning::QConstraint> _constraint;
		rw::common::Ptr<rw::pathplanning::QSampler> _sampler;
        double _resolution;

		rw::common::Ptr<rw::pathplanning::QEdgeConstraint> _edge;

        std::pair<rw::math::Q, rw::math::Q> _bounds;

        rw::math::Q _metricWeights;
		rw::math::QMetric::Ptr _metric;

        double _Rneighbor;
        size_t _Nneighbor;
        NeighborSearchStrategy _neighborSearchStrategy;
        size_t _partialIndexTableDimensions;
        CollisionCheckingStrategy _collisionCheckingStrategy;

        ShortestPathSearchStrategy _shortestPathSearchStrategy;
        double _astarTimeOutTime;

        // for stats testing
        rw::common::Timer collisionTimer;
        rw::common::Timer roadmapBuildTimer;
        rw::common::Timer queryTimer;
        rw::common::Timer shortestPathTimer;
        rw::common::Timer neighborTimer;
        rw::common::Timer enhanceTimer;


        /**
         * @brief The data contained in the PRM graph node
         */
        class NodeData{
        public:
            //! Joint configuration
            rw::math::Q q;

            //! Has the node been checked for collision
            bool checked;
        private:
            friend class boost::serialization::access;
            // When the class Archive corresponds to an output archive, the
            // & operator is defined similar to <<.  Likewise, when the class Archive
            // is a type of input archive the & operator is defined similar to >>.
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & q;
                ar & checked;
            }

        };

        /**
         * @brief The data contained in the PRM graph edge
         */
        class EdgeData{
        public:
            //! The edge-weight (defined as pPath(left, right))
            double weight;

            /**
             * @brief The resolution to which this node has been checked
             * (if resolution < _resolution then the edge is considered checked)
             */
            double resolution;

            rw::math::Q q1;
            rw::math::Q q2;
        private:
            friend class boost::serialization::access;
            // When the class Archive corresponds to an output archive, the
            // & operator is defined similar to <<.  Likewise, when the class Archive
            // is a type of input archive the & operator is defined similar to >>.
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                ar & weight;
                ar & resolution;
                ar & q1;
                ar & q2;
            }
        };

        /**
         * @brief The PRM (Probabilistic RoadMap)
         */
        typedef boost::adjacency_list<boost::listS,
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
        rwlibs::algorithms::KDTreeQ<Node>::Ptr  _kdtree;
        std::list<const rwlibs::algorithms::KDTreeQ<Node>::KDNode*> _kdnodesSearchResult;

        void initialize(
            const rw::models::Device& device,
            const rw::kinematics::State& state);

		bool inCollision(const rw::math::Q& a, const rw::math::Q& b) const;
        bool addEdge(Node n1, Node n2, double dist);

        void addEdges(Node node);

        Node addNode(const rw::math::Q& q, bool checked);

        double estimateRneighbor(size_t roadmapsize);

        bool searchForShortestPathDijkstra(
            const Node& nInit, const Node& nGoal, std::list<Node>& result);
        bool searchForShortestPathAstar(
            const Node& nInit, const Node& nGoal, std::list<Node>& result);

        bool inCollision(std::list<Node>& path);

        bool enhanceEdgeCheck(Edge& e);
        void removeCollidingNode(Node node);
        void removeCollidingEdge(Edge edge);

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
            PathHeuristic(PRM& prm, const rw::math::QMetric* metric, const Node& nGoal) :
                _prm(prm),
                _metric(metric),
                _qGoal(prm[nGoal].q)
            {}

            /**
             * @brief Calculates distance from n to goal
             * @param n [in] the node n
             * @return the distance
             */
            double operator()(const Node& n)
            {
                return _metric->distance(_qGoal, _prm[n].q);
            }

        private:
            PRM& _prm;
            //! Controlling pathplanner
            const rw::math::QMetric* _metric;
            //! Goal configuration
            rw::math::Q _qGoal;
        };
    };

	/* @} */
}} // end namespaces

#endif // end include guard
