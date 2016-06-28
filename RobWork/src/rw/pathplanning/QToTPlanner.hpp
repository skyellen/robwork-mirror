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

#include <rw/math/Metric.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace pathplanning {
	class QIKSampler;
	class QToQSamplerPlanner;
	class QToQPlanner;

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Approach planner interface.

       An approach planner plans a path from a configuration for the device to a
       configuration for the tool.
    */
    class QToTPlanner : public PathPlanner<rw::math::Q, const rw::math::Transform3D<> >
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QToTPlanner> Ptr;

        /**
           @brief An approach planner for a sampler of IK solutions and a region
           planner.

           Target configurations are sampled by \b ikSampler and fed to \b
           planner.

           @param planner [in] Planner for a QSampler region.
           @param ikSampler [in] Sampler of IK solutions for the target transform.
        */
		static QToTPlanner::Ptr make(
			rw::common::Ptr<QToQSamplerPlanner> planner,
			rw::common::Ptr<QIKSampler> ikSampler);

        /**
           @brief An approach planner for a standard path planner and a sampler
           of IK solutions.

           For each query(from, to) call, the planner extracts \b cnt samples
           from \b sampler and calls \b planner with the configuration closest
           to \b from according to \b metric.
        */
        static
			QToTPlanner::Ptr makeToNearest(
			rw::common::Ptr<QToQPlanner> planner,
			rw::common::Ptr<QIKSampler> sampler,
			rw::math::QMetric::Ptr metric,
            int cnt);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
