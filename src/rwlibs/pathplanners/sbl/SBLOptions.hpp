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

#ifndef rwlibs_pathplanners_sbl_SBLOptions_HPP
#define rwlibs_pathplanners_sbl_SBLOptions_HPP

/**
   @file SBLOptions.hpp
*/

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QExpand.hpp>
#include <rw/models/Device.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief SBL planner setup.

       SBLOptions is the value stored in SBLSetup.

       SBLOptions is a seperate file so that we can keep SBLSetup as abstract as
       possible.

       SBLOptions is used by SBLInternal and is for internal use only.
    */
    class SBLOptions
    {
    public:
        SBLOptions(
            const rw::pathplanning::PlannerConstraint& constraint,
            rw::pathplanning::QExpandPtr expansion,
            rw::math::MetricPtr metric,
            double connectRadius);

        rw::pathplanning::PlannerConstraint constraint;
        rw::pathplanning::QExpandPtr expansion;
        rw::math::MetricPtr metric;
        double connectRadius;

        enum NearNodeSelection {
            UniformSelect,
            UniformFromCell,
            NearestFromCell,
            NearestNode
        };

        int resetCount;
        int rootSampleInterval;
        double nodesPerCell;
        NearNodeSelection nearNodeSelection;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
