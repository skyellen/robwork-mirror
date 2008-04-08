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

#ifndef rwlibs_pathplanners_rrt_RRTPathPlanner_HPP
#define rwlibs_pathplanners_rrt_RRTPathPlanner_HPP

/**
 * @file RRTPathPlanner.hpp
 */

#include <rw/pathplanning/PathPlanner.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/StraightLinePathPlanner.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include <vector>
#include <list>
#include <cmath>
#include <limits.h>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
     * @brief A path planner based on the "Rapidly expanding Random Tree's.
     *
     * The principle of the algorithm as described in the paper "RRT-Connect: 
     * An Efficient Approach to Single-Query Path Planning".
     */
    class RRTPathPlanner : public rw::pathplanning::PathPlanner
    {
    public:
        /**
         * @brief Constructs the path planner
         *
         * @param workcell [in] the workcell   
         * @param device [in] the device to plan for
         * @param detector [in] the collision detector to use
         * @param state [in] State associated with the workcell
         * @param resolution [in] the resolution to use for collision checking
         */
        RRTPathPlanner(rw::models::WorkCell* workcell,
                       rw::models::Device* device,
                       rw::proximity::CollisionDetector* detector,
                       const rw::kinematics::State& state,
                       double resolution);

      
        /**
         * @copydoc rw::pathplanning::PathPlanner::query
         */
        bool query(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            rw::pathplanning::Path& path,
            double timeS);

    private:
        /**
         * @brief Forward decleration
         */
        class Node;

    protected:
        /**
         * @brief A RRT-Tree
         */
        typedef std::vector<Node*> Tree;

        /**
         * @brief Result from extend and connect
         */
        enum ExtendResult{
            //! Trapped
            TRAPPED,
            //! Reached
            REACHED,
            //! Advanced
            ADVANCED
        };

        /**
         * @brief Finds nearest neighbor
         *
         * @param tree [in] An RRT-Tree
         *
         * @param q [in] a joint configuration
         *
         * @return the nearest neighbor of q in the tree
         *
         * @pre tree is not empty
         */
        const Node* nearestNeighbor(const Tree* const tree, const rw::math::Q& q) const;

        /**
         * @brief Tries to extend the tree from qNearNode towards q
         *
         * @param tree [in] the tree
         *
         * @param q [in] the joint configuration to extends towards
         *
         * @param qNearNode [in] the node to extend from
         *
         * @return
         * - REACHED if q was reached,
         * - ADVANCED if a step was made towards q but q was not reached,
         * - TRAPPED if a step towards q could not be made due to collision
         */
        ExtendResult extend(Tree* const tree, const rw::math::Q& q, const Node* qNearNode);

        /**
         * @brief Tries to connect q to the nearest node in tree
         *
         * @param tree [in] the tree to connect to
         *
         * @param q [in] the joint configuration
         *
         * @return REACHED if q was reached or TRAPPED if a connection could not
         * be made due to collision
         */
        ExtendResult connect(Tree* const tree, const rw::math::Q& q);

        /**
         * @brief Returns the distance between @f$ \mathbf{q}_1 @f$ and @f$
         * \mathbf{q}_2 @f$: @f$ d(\mathbf{q}_1, \mathbf{q}_2) @f$
         *
         * @param q1 [in] @f$ \mathbf{q}_1 @f$
         * @param q2 [in] @f$ \mathbf{q}_2 @f$
         * @return the distance between @f$ \mathbf{q}_1 @f$ and @f$ \mathbf{q}_2 @f$
         *
         * This is the distance function used be the nearestNeighbor() method.
         *
         * The distance is currently defined as the euclidian norm of the
         * difference between @f$ \mathbf{q}_1 \f$ and \f$ \mathbf{q}_2 @f$ @f$
         * d(\mathbf{q}_1, \mathbf{q}_2) = (\Sigma_i^n q_i^2)^{1/2} @f$
         */
        double d(const rw::math::Q& q1, const rw::math::Q& q2) const;

        /**
         * @brief Checks to see if the path between qNear and qNew is in collision
         * @param qNear [in] a joint configuration
         * @param qNew [in] a joint configuration
         * @return true if a collision was found, false otherwise
         */
        bool collides(const rw::math::Q& qNear,const rw::math::Q& qNew);

    private:
        //! A Simple RRT-Tree node
        class Node{
        public:
            //! Constructs the object
            Node(const rw::math::Q& config, const Node* parent) :
                _config(config), _parent(parent){}

            //! Returns the joint configuration
            const rw::math::Q& getQ() const{
                return _config;
            }

            //! Returns the parent node or NULL if this is the root node in the tree
            const Node* getParent() const{
                return _parent;
            }

        private:
            rw::math::Q _config;
            const Node* _parent;
        };

        rw::pathplanning::PlannerUtil _utils;
        rw::models::Device *_device;

        //! The resolution to use for collision checking
        double _resolution;

        //! The localplanner used for making advancements toward nodes
        rw::pathplanning::StraightLinePathPlanner _lineplanner;

        //! The maximum number of iterations
        const static unsigned int K;

        //! The stepsize to use
        const static double EPSILON;
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
