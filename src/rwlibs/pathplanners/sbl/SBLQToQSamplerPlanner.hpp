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

#ifndef rwlibs_pathplanners_sbl_SBLQToQSamplerPlanner_HPP
#define rwlibs_pathplanners_sbl_SBLQToQSamplerPlanner_HPP

/**
 * @file SBLQToQSamplerPlanner.hpp
 */

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QEdgeConstraint.hpp>
#include <rw/pathplanning/QExpand.hpp>
#include <rw/pathplanning/QSampler.hpp>

#include <rw/pathplanning/QToQSamplerPlanner.hpp>
#include <rw/models/Device.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Sampled region planner in the style of the SBL planner of Gildardo
       Sanches and Jean-Claude Latombe.

       To construct a standard SBL path planner, see the QToQPlanner::make()
       region planner to path planner conversion function.
    */
    class SBLQToQSamplerPlanner : public rw::pathplanning::QToQSamplerPlanner
    {
    public:
        /**
           @brief An SBL region planner for a given constraint, edge planner and
           node expansion strategy.
        */
        static std::auto_ptr<QToQSamplerPlanner> make(
            rw::pathplanning::QConstraintPtr constraint,
            rw::pathplanning::QEdgeConstraintPtr edge,
            rw::pathplanning::QExpandPtr expansion);

        /**
           @brief An SBL planner for a constraint, edge planner, and device.

           A suitable default expansion strategy is chosen for the device.
        */
        static std::auto_ptr<QToQSamplerPlanner> make(
            rw::pathplanning::QConstraintPtr constraint,
            rw::pathplanning::QEdgeConstraintPtr edge,
            rw::models::DevicePtr device);

        /**
           @brief An SBL planner for a constraint and device.

           A suitable default expansion strategy and edge planner is chosen for
           the device.
        */
        static std::auto_ptr<QToQSamplerPlanner> make(
            rw::pathplanning::QConstraintPtr constraint,
            rw::models::DevicePtr device);

        /**
           @brief Destructor
        */
        ~SBLQToQSamplerPlanner();

    private:

        // The primitive constructor to which the make() functions forward.
        SBLQToQSamplerPlanner(
            rw::pathplanning::QConstraintPtr constraint,
            rw::pathplanning::QEdgeConstraintPtr edge,
            rw::pathplanning::QExpandPtr expansion);

        bool doQuery(
            const rw::math::Q& from,
            rw::pathplanning::QSampler& to,
            rw::pathplanning::Path& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        class Impl;
        Impl* _impl;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
