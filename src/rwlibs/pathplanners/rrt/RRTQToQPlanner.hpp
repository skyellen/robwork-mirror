/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_RRT_RRTQTOQPLANNER_HPP

/**
   @file RRTQToQPlanner.hpp
*/

#include "RRTTree.hpp"
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/math/Metric.hpp>

#include <vector>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief A path planner based on Rapidly Expanding Random Trees.

       The algorithm as described in the paper "RRT-Connect: An Efficient
       Approach to Single-Query Path Planning" by James J. Kuffner and Steven M.
       LaValle.
    */
    class RRTQToQPlanner : public rw::pathplanning::QToQPlanner
    {
    public:
        /**
           @brief Constructor

           Edges are verified by \b edge.

           @param constraint [in] Constraint for configurations and edges.

           @param sampler [in] Sampler of the configuration space.

           @param metric [in] Metric for nearest neighbor search.

           @param extend [in] Distance measured by \b metric by which to extend
           the tree towards an attractor configuration.
        */
        RRTQToQPlanner(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QSamplerPtr sampler,
            rw::math::QMetricPtr metric,
            double extend);

    private:
        typedef rw::trajectory::QPath Path;
        typedef RRTNode<rw::math::Q> Node;
        typedef RRTTree<rw::math::Q> Tree;

        bool doQuery(
            const rw::math::Q& start,
            const rw::math::Q& goal,
            Path& path,
            const rw::pathplanning::StopCriteria& stop);

        enum ExtendResult { Trapped, Reached, Advanced };

        ExtendResult extend(Tree& tree, const rw::math::Q& q, Node* qNearNode);
        ExtendResult connect(Tree& tree, const rw::math::Q& q);

    private:
        rw::pathplanning::PlannerConstraint _constraint;
        rw::pathplanning::QSamplerPtr _sampler;
        rw::math::QMetricPtr _metric;
        double _extend;
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
