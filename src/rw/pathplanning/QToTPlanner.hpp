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

#ifndef rw_pathplanning_QToTPlanner_HPP
#define rw_pathplanning_QToTPlanner_HPP

/**
   @file QToTPlanner.hpp
*/

#include "PathPlanner.hpp"
#include "QToQSamplerPlanner.hpp"

#include <rw/invkin/IterativeIK.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    class QToTPlanner;

    //! A pointer to an QToTPlanner.
    typedef rw::common::Ptr<QToTPlanner> QToTPlannerPtr;

    /**
       @brief Approach planner interface.

       An approach planner plans a path from a configuration for the device to a
       configuration for the tool.
    */
    class QToTPlanner : public PathPlanner<const rw::math::Transform3D<> >
    {
    public:
        /**
           @brief Construct an approach planner from a region planner.

           The approach planner takes as target in the query() call a \b
           baseTend transform for \b device.

           @param planner [in] Planner for a QSampler region.
           @param solver [in] Iterative IK solver for \b device.
           @param device [in] Device for which IK is solved.
           @param state [in] State of the rest of the workcell.
           @param maxAttempts [in] Number of seeds to use per \b solver attempt.
        */
        static std::auto_ptr<QToTPlanner> make(
            QToQSamplerPlannerPtr planner,
            rw::invkin::IterativeIKPtr solver,
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            int maxAttempts);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
