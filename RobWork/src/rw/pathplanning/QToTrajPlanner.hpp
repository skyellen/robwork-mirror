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


#ifndef RW_PATHPLANNING_QTOTRAJPLANNER_HPP
#define RW_PATHPLANNING_QTOTRAJPLANNER_HPP

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

    /**
       @brief Interface for planning robot trajectories with tool path trajectory constraints

       An approach planner plans a path from a configuration for the device to a
       configuration for the tool.
    */
    class QToTrajPlanner : public PathPlanner<rw::math::Q, rw::trajectory::Transform3DTrajectory::Ptr >
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QToTrajPlanner> Ptr;

        /**
           @brief An approach planner for a sampler of IK solutions and a region
           planner.

           Target configurations are sampled by \b ikSampler and fed to \b
           planner.

           @param planner [in] Planner for a QSampler region.
           @param ikSampler [in] Sampler of IK solutions for the target transform.
        */
		static QToTPlanner::Ptr make(
			QToQSamplerPlanner::Ptr planner,
			QIKSampler::Ptr ikSampler);

        /**
           @brief An approach planner for a standard path planner and a sampler
           of IK solutions.

           For each query(from, to) call, the planner extracts \b cnt samples
           from \b sampler and calls \b planner with the configuration closest
           to \b from according to \b metric.
        */
        static
			QToTPlanner::Ptr makeToNearest(
			QToQPlanner::Ptr planner,
			QIKSampler::Ptr sampler,
			rw::math::QMetric::Ptr metric,
            int cnt);

    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::pathplanning::QToTPlanner::Factory,rw::pathplanning::QToTPlanner,rw::pathplanning::QToTPlanner}
    	 */

    	/**
    	 * @brief a factory for QToTPlanner. This factory also defines an
    	 * extension point for QToTPlanner. This permit users to add
    	 * QToQPlanners that will be available through this factory
    	 */
        class Factory: public rw::common::ExtensionPoint<QToTPlanner> {
        public:
        	//! constructor
            Factory():rw::common::ExtensionPoint<QToTPlanner>("rw::pathplanning::QToQPlanner", "Extension point for QToTPlanner."){};

            /**
             * @brief get a specific planner based on id
             * @param id [in] string identifier of planner library
             * @return a QToTPlanner if matching id exists else NULL
             */
            static rw::common::Ptr<QToTPlanner> getPlanner(const std::string& id);

            /**
             * @brief get all avaliable QToQPlanner's
             * @return all avalilable QToQPlanners
             */
            static std::vector<QToTPlanner::Ptr> getPlanners();

            /**
             * @brief get a list of supported planners
             * @return
             */
            static std::vector<std::string> getPlannerIDs();

        };

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
