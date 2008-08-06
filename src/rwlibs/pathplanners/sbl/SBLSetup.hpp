/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_pathplanners_sbl_SBLSetup_HPP
#define rwlibs_pathplanners_sbl_SBLSetup_HPP

/**
   @file SBLSetup.hpp
*/

#include "SBLOptions.hpp"
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QExpand.hpp>
#include <rw/models/Device.hpp>

namespace rwlibs { namespace pathplanners {

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

           @param expansion [in] Expansion strategy for insertion of new nodes.
           The nodes returned by the expansion strategy must be collision free,
           and the expansion strategy must always return a non-empty
           configuration.

           @param metric [in] Distance metric for nearest neighbor searching.

           @param connectRadius [in] Attempt connection of the trees if the
           distance to the nearest neighbor is below this threshold.
        */
        static
        SBLSetup make(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QExpandPtr expansion,
            rw::math::QMetricPtr metric,
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

           @param constraint [in] Planning constraint.

           @param device [in] Device for which planning is done.

           @param expandRadius [in] Node expansion radius.

           @param expandRadius [in] Neighbor connection radius.
        */
        static
        SBLSetup make(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::models::DevicePtr device,
            double expandRadius = 0.2,
            double connectRadius = 1.0);

    private:
        SBLSetup(const SBLOptions& options) : options(options) {}

    public:
        SBLOptions options;
        friend class SBLInternal;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
