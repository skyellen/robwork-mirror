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
#include "QToQPlanner.hpp"
#include "QToQSamplerPlanner.hpp"
#include "QIKSampler.hpp"

#include <rw/math/Metric.hpp>
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
           @brief An approach planner for a sampler of IK solutions and a region
           planner.

           Target configurations are sampled by \b ikSampler and fed to \b
           planner.

           @param planner [in] Planner for a QSampler region.
           @param ikSampler [in] Sampler of IK solutions for the target transform.
        */
        static QToTPlannerPtr make(
            QToQSamplerPlannerPtr planner,
            QIKSamplerPtr ikSampler);

        /**
           @brief An approach planner for a standard path planner and a sampler
           of IK solutions.

           For each query(from, to) call, the planner extracts \b cnt samples
           from \b sampler and calls \b planner with the configuration closest
           to \b from according to \b metric.
        */
        static
        QToTPlannerPtr makeToNearest(
            QToQPlannerPtr planner,
            QIKSamplerPtr sampler,
			rw::math::QMetricPtr metric,
            int cnt);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
