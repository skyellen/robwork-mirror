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


#ifndef RWLIBS_PATHPLANNERS_SBL_SBLPLANNER_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLPLANNER_HPP

/**
   @file SBLPlanner.hpp
*/

#include "SBLSetup.hpp"
#include <rw/pathplanning/QToTPlanner.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QToQSamplerPlanner.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief SBL based planners.

       @relates QToQPlanner
       @relates QToTPlanner
       @relates QToQSamplerPlanner
    */
    class SBLPlanner
    {
    public:
    	//! @brief Smart pointer type for SBLPlanner.
        typedef rw::common::Ptr<SBLPlanner> Ptr;

        /**
           @brief An SBL based sampled region planner.

           @param setup [in] Setup for the planner.
        */
		static rw::pathplanning::QToQSamplerPlanner::Ptr makeQToQSamplerPlanner(const SBLSetup& setup);

        /**
           @brief An SBL based point-to-point planner.

           @param setup [in] Setup for the planner.
        */
		static rw::pathplanning::QToQPlanner::Ptr makeQToQPlanner(const SBLSetup& setup);

        /**
           @brief An SBL based point-to-tool-position planner.

           @param setup [in] Setup for the planner.
           @param ikSampler [in] Sampler of IK solutions for the target transform.
        */
		static rw::pathplanning::QToTPlanner::Ptr makeQToTPlanner(
            const SBLSetup& setup,
			rw::common::Ptr<rw::pathplanning::QIKSampler> ikSampler);

    private:
        SBLPlanner();
        SBLPlanner(const SBLPlanner&);
        SBLPlanner& operator=(const SBLPlanner&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
