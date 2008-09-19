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

#ifndef rw_pathplanning_QToQSamplerPlanner_HPP
#define rw_pathplanning_QToQSamplerPlanner_HPP

/**
   @file QToQSamplerPlanner.hpp
*/

#include "PathPlanner.hpp"
#include "QSampler.hpp"

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    class QToQSamplerPlanner;

    //! A pointer to a QToQSamplerPlanner.
    typedef rw::common::Ptr<QToQSamplerPlanner> QToQSamplerPlannerPtr;

    /**
       @brief Sampled region planner interface.

       QToQSamplerPlanner plans a configuration space path from a start
       configuration to any configuration in the set represented by a sampler.
    */
    class QToQSamplerPlanner : public PathPlanner<rw::math::Q, QSampler>
    {};

    /*@}*/
}} // end namespaces

#endif // end include guard
