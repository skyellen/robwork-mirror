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


#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP
#define RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP

#include <rw/common/Ptr.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rwlibs {
namespace pathoptimization{

/** @addtogroup pathoptimization */
/*@{*/


/**
 * @brief Interface for ClearanceCalculator
 *
 * A ClearanceCalculator provides a mean for calculating the clearance of for a state. While the
 * concept of clearance usually refers to the distance between a device and obstacle, not such assumption
 * should be made based on the interface, as other fitness criteria may be implemented.
 *
 * Only convention is that a high clearance value is better than a low.
 */
class ClearanceCalculator
{
public:
	/**
	 * @brief Destructor
	 */
    virtual ~ClearanceCalculator();

    /**
     * @brief Calculates Clearance for the state
     *
     * @param state [in] State for which to calculate the clearance
     * @return The clearance.
     */
	virtual double clearance(rw::kinematics::State& state) = 0;
};

/**
 * @brief Pointer to a ClearanceCalculator
 */
typedef rw::common::Ptr<ClearanceCalculator> ClearanceCalculatorPtr;

/* @} */

} //end namespace pathoptimization
} //end namespace rwlibs

#endif //#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP
