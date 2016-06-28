/*******************************************************************************
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


#ifndef RW_PATHPLANNING_QTOQSAMPLERPLANNER_HPP
#define RW_PATHPLANNING_QTOQSAMPLERPLANNER_HPP

/**
   @file QToQSamplerPlanner.hpp
*/

#include "PathPlanner.hpp"
#include "QSampler.hpp"

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Sampled region planner interface.

       QToQSamplerPlanner plans a configuration space path from a start
       configuration to any configuration in the set represented by a sampler.
    */
    class QToQSamplerPlanner : public PathPlanner<rw::math::Q, QSampler>
    {
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QToQSamplerPlanner> Ptr;

	};

    /*@}*/
}} // end namespaces

#endif // end include guard
