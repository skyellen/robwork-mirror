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


#ifndef RWLIBS_PATHPLANNERS_SBL_SBLSETUP_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLSETUP_HPP

/**
   @file SBLSetup.hpp
*/

#include "SBLOptions.hpp"

namespace rw { namespace models { class Device; } }

namespace rwlibs { namespace pathplanners {
class SBLExpand;

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Common parameters for SBL based planners.

       All versions of the SBL planner base verify configurations and paths in
       the configuration space using a PlannerConstraint object.

       In addition, parameters can given to define how expansion around a node
       of the tree should be done and under what circumstances the two trees
       should be connected.

       A SBLSetup object stores pointers to the shared objects, but can be
       copied and assigned freely.
    */
    class SBLSetup
    {
    public:
        /**
           @brief Constructor

           The SBL planner for this setup performs brute force search for the
           nearest neighbor of the other tree, and attempts to connect the trees
           if the distance to the neighbor is below a given threshold.

           @param constraint [in] Planning constraint.

           @param edgeConstraint [in] Planning constraint for edges.

           @param expansion [in] Expansion strategy for insertion of new nodes.
           The nodes returned by the expansion strategy must be collision free
           or empty. If an empty configuration is returned, the planner tries to
           expand somewhere else.

           @param metric [in] Distance metric for nearest neighbor searching.

           @param connectRadius [in] Attempt connection of the trees if the
           distance to the nearest neighbor is below this threshold.
        */
        static
        SBLSetup make(
			rw::pathplanning::QConstraint::Ptr constraint,
			rw::pathplanning::QEdgeConstraintIncremental::Ptr edgeConstraint,
            rw::common::Ptr<SBLExpand> expansion,
			rw::math::QMetric::Ptr metric,
            double connectRadius);

        /**
           @brief Constructor

           Simple default expansion and tree connection strategies are chosed
           based on the device for which the planning is done.

           The planner expands uniformly at random with a maximum stepsize of \b
           expandRadius relative to the diameter of the configuration space. The
           step size and the diameter is measured by the infinity metric.

           The planner connect a newly created node to the nearest node of the
           other tree if the distance to the other node (measured by the
           infinity metric and relative to the diameter of the configuration
           space) is less than \b connectRadius.

           If \b expandRadius or \b connectRadius is negative, a default value
           is chosen.

           @param constraint [in] Planning constraint.

           @param edgeConstraint [in] Planning constraint for edges.

           @param device [in] Device for which planning is done.

           @param expandRadius [in] Node expansion radius.

           @param connectRadius [in] Neighbor connection radius.
        */
        static
        SBLSetup make(
			rw::pathplanning::QConstraint::Ptr constraint,
			rw::pathplanning::QEdgeConstraintIncremental::Ptr edgeConstraint,
//            const rw::pathplanning::PlannerConstraint& constraint,
			rw::common::Ptr<rw::models::Device> device,
            double expandRadius = -1,
            double connectRadius = -1);

    private:
        SBLSetup(const SBLOptions& options) : options(options) {}

    public:
        //! @brief Internal options to use.
        SBLOptions options;
        friend class SBLInternal;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
