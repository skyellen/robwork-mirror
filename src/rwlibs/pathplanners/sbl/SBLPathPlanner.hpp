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

#ifndef rwlibs_pathplanners_sbl_SBLPathPlanner_HPP
#define rwlibs_pathplanners_sbl_SBLPathPlanner_HPP

/**
 * @file SBLPathPlanner.hpp
 */

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/PathPlanner.hpp>
#include <rw/models/Device.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       An implementation of the SBL path planner of Gildardo Sanches and
       Jean-Claude Latombe.
    */
    class SBLPathPlanner : public rw::pathplanning::PathPlanner
    {
    public:
        /**
           Path planner for the configuration space for which the normalizer is
           \b normalizer and the collision checker is \b constraint.
        */
        SBLPathPlanner(
            rw::pathplanning::QConstraintPtr constraint,
            const rw::pathplanning::QNormalizer& normalizer);

        /**
           @copydoc rw::pathplanning::PathPlanner::solve
        */
        bool solve(
            const rw::math::Q& from,
            const rw::math::Q& to,
            rw::pathplanning::Path& path,
            rw::pathplanning::StopCriteriaPtr stop);

        /**
           Destructor
        */
        ~SBLPathPlanner();

    private:
        class Impl;
        Impl* _impl;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
