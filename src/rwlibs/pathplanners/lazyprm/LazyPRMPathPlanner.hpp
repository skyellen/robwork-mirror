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

#ifndef rwlibs_pathplanners_lazyprm_LazyPRMPathPlanner_HPP
#define rwlibs_pathplanners_lazyprm_LazyPRMPathPlanner_HPP

/**
 * @file LazyPRMPathPlanner.hpp
 */

#include <rw/pathplanning/StraightLinePathPlanner.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

namespace rwlibs { namespace pathplanners{

    /** @addtogroup pathplanners */
    /*@{*/

    /**
     * @brief A PRM path planner based on the article "A Lazy Probabilistic Roadmap Planner for Single Query Path Planning"
     *
     * @todo this pathplanner is not finished, the following needs to be done:
     * - Decide whether to use normalized joint coordinates or "real" joint coordinates
     * - Implement the bisection line segment checker (the line checker currently just interpolates from start to finish)
     * - Find methods to calculate the metric weights pCollWeight and pPathWeight
     * - Find formula for Rneighbor
     */
    class LazyPRMPathPlanner : public rw::pathplanning::PathPlanner {
    public:
        /**
         * @brief Creates LazyPRM path planner object
         * @param workcell [in] the workcell
         * @param device [in] the device to plan for
         * @param detector [in] the collision detector to use
         * @param resolution [in] the interpolation resolution to use when performing collision detection
         */
        LazyPRMPathPlanner(
            rw::models::WorkCell* workcell,
            rw::models::DeviceModel* device,
            rw::proximity::CollisionDetector* detector,
            double resolution);

        /**
         * @brief Destroys object
         */
        ~LazyPRMPathPlanner();

        /**
         * @brief Initializes and resets roadmap
         *
         * @brief device [in] the device to plan for
         */
        void initialize(rw::models::DeviceModel* device);

        /**
         * @copydoc rw::pathplanning::PathPlanner::query
         */
        bool query(const Q& qInit, const Q& qGoal, rw::pathplanning::Path& path, double timeS = 60.0);

    private:

        rw::pathplanning::PlannerUtil _utils;

        rw::models::DeviceModel* _device;

        /**
         * @brief Calculates the cost of moving from configuration @f$ \mathbf{q}_1 @f$ to @f$ \mathbf{q}_2 @f$: @f$ \rho_{path}(\mathbf{q}_1, \mathbf{q}_2) @f$
         * @param q1 [in] @f$ \mathbf{q}_1 @f$
         * @param q2 [in] @f$ \mathbf{q}_2 @f$
         * @return the cost of moving from @f$ \mathbf{q}_1 @f$ to @f$ \mathbf{q}_2 @f$
         */
        double pPath(const Q& q1, const Q& q2) const;

        /**
         * @brief Calculates the difficulty of making a collision free connection between \f$ \mathbf{q}_1 \f$ and \f$ \mathbf{q}_2 \f$: \f$ \rho_{coll}(\mathbf{q}_1, \mathbf{q}_2) \f$
         * @param q1 [in] \f$ \mathbf{q}_1 \f$
         * @param q2 [in] \f$ \mathbf{q}_2 \f$
         * @return the estimated difficulty of making a collision free connection between \f$ \mathbf{q}_1 \f$ to \f$ \mathbf{q}_2 \f$
         */
        double pColl(const Q& q1, const Q& q2) const;

        /**
         * @brief The data contained in the PRM graph node
         */
        struct NodeData{
            //! Joint configuration
            Q q;

            //! Is the node created by random or by seeding
            bool random;

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
        };

        /**
         * @brief The PRM (Probabilistic RoadMap)
         */
        typedef boost::adjacency_list<
            boost::listS,
            boost::listS,
            boost::undirectedS,
            /*boost::property<boost::vertex_index_t, std::size_t,*/
            NodeData /*>*/,
            EdgeData> PRM;

        //! A PRM node
        typedef PRM::vertex_descriptor Node;

        //! A PRM edge
        typedef PRM::edge_descriptor Edge;

        //! The list of seed points to distribute new nodes about (used in the node enhancement step)
        std::vector<rw::math::Q> _seeds;

        /**
         * @brief Searches for the shortest path between nInit and nGoal
         * @param nInit [in] the initial node
         * @param nGoal [in] the goal node
         * @param path [out] the shortest path (if a path exists)
         * @return true if a path between nInit and nGoal was found, false if there is no
         * path between nInit and nGoal
         */
        bool searchForShortestPath(Node nInit, Node nGoal, std::list<Node>& path);

        /**
         * @brief Checks path for collision
         * @param path [in] the path to check
         * @return true if the path is okay (no-collision was found), false otherwise
         */
        bool checkPathForCollision(const std::list<Node>& path);

        /**
         * @brief Node enhancement method
         */
        void doNodeEnhancement();

        /**
         * @brief Removes colliding node
         * @param node [in] the node to remove
         */
        void removeCollidingNode(const Node node);

        /**
         * @brief Removes colliding edge
         * @param edge [in] the edge to remove
         */
        void removeCollidingEdge(const Edge edge);

        //! Number of initial nodes
        unsigned int Ninit;

        //! Number of nodes to add in each enhancement step
        unsigned int Nenh;

        //! Expected number of average connections for each node
        unsigned int Mneighb;

        //! Radius of k dimensional hypersphere covering nearest neighbor
        double Rneighb;

        //! Local planner (in this case a straightline planner)
        rw::pathplanning::StraightLinePathPlanner _localplanner;

        //! Has initializeRoadmap been called
        bool _initialized;

        /**
         * @brief Adds configuration to roadmap and connects it to nearby nodes
         * @param q [in] the configuration
         * @param random [in] whether the configuration was selected randomly or around a seed
         * @return the appended node
         */
        Node addNode(const rw::math::Q& q, bool random);

        //! EnhanceStep
        void enhanceNode();

        //! The roadmap
        PRM _graph;

        //! Resolution used for doing collision checks
        double _resolution;

        //! w0, w1, .... wk weights used in pColl
        rw::math::Q* _pCollWeights;

        //! w0, w1, .... wk weights used in pPath
        rw::math::Q* _pPathWeights;

        /**
         * @brief The heuristic distance meassure used by the A* shortest path search
         */
        class pPathHeuristic : public boost::astar_heuristic<PRM, double>{
        public:
            /**
             * @brief Creates object
             * @param lazy [in] the lazy PRM path planner
             * @param nGoal [in] the goal node
             */
            pPathHeuristic(const LazyPRMPathPlanner* lazy, const Node& nGoal) :
                _nGoal(nGoal),
                _lazy(lazy)
            {
            }

            /**
             * @brief Calculates distance from n to goal
             * @param n [in] the node n
             * @return the distance
             */
            double operator()(const Node& n){
                return _lazy->pPath(
                    _lazy->_graph[_nGoal].q,
                    _lazy->_graph[n].q
                    );
            }

        private:
            //! Goal configuration
            Node _nGoal;

            //! Controlling pathplanner
            const LazyPRMPathPlanner* _lazy;
        };

    };

    /*\}*/
}} // end namespaces

#endif // end include guard
