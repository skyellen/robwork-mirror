/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP
#define RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP

#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

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
    virtual ~ClearanceCalculator() {};

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
