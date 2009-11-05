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


#ifndef RW_PATHPLANNING_QTOTPLANNER_HPP
#define RW_PATHPLANNING_QTOTPLANNER_HPP

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
    class QToTPlanner : public PathPlanner<rw::math::Q, const rw::math::Transform3D<> >
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
