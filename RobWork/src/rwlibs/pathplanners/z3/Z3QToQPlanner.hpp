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


#ifndef RWLIBS_PATHPLANNERS_Z3_Z3QTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_Z3_Z3QTOQPLANNER_HPP

/**
   @file Z3QToQPlanner.hpp
*/

#include <rw/pathplanning/QToQPlanner.hpp>

namespace rw { namespace pathplanning { class QSampler; } }

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Z3 based point-to-point planner.
    */
    class Z3QToQPlanner : public rw::pathplanning::QToQPlanner
    {
    public:
        /**
           @brief Constructor

           @param sampler [in] Sampler of the configuration space.

           @param localPlanner [in] Local planner for connecting the configurations.

           @param nodeCnt [in] Number of supporting configurations to insert.
           If \b nodeCnt is negative, a default value is chosen.

           @param repeatCnt [in] Number of times to repeat the attempt. If \b
           repeatCnt is negative, the attempts are repeated forever (or until
           the stop criteria returns true).
        */
        Z3QToQPlanner(
        	rw::common::Ptr<rw::pathplanning::QSampler> sampler,
			rw::pathplanning::QToQPlanner::Ptr localPlanner,
            int nodeCnt,
            int repeatCnt);

    private:

        bool doQuery(
            const rw::math::Q& start,
            const rw::math::Q& goal,
			rw::trajectory::QPath& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        rw::common::Ptr<rw::pathplanning::QSampler> _sampler;
		rw::pathplanning::QToQPlanner::Ptr _localPlanner;
        int _nodeCnt;
        int _repeatCnt;
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
