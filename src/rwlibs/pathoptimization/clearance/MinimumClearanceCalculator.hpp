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

#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP
#define RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP

#include "ClearanceCalculator.hpp"

#include <boost/shared_ptr.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>

namespace rwlibs {
namespace pathoptimization {

/** @addtogroup pathoptimization */
/*@{*/

/**
 * @brief Implements a MinimumClearanceCalculator
 *
 * The minimum clearance is defined as the minimal distance between any two geometries, which are not excluded
 * by the collision setup.
 */
class MinimumClearanceCalculator: public ClearanceCalculator
{
public:
    /**
     * @brief Constructs a MinimumClearanceCalculator using the \b DistanceCalculator provided.
     *
     * Use this constructor to use an already existing DistanceCalculator
     *
     * @param distancecalculator [in] The distance calculator to use
     */
	MinimumClearanceCalculator(rw::proximity::DistanceCalculatorPtr distancecalculator);

	/**
	 * @brief Constructs a MinimumClearanceCalculator for a workcell
	 * @param workcell [in] WorkCell for which to calculate the minimum clearance
	 * @param state [in] State of the workcell
	 */
	MinimumClearanceCalculator(rw::models::WorkCellPtr workcell,
	                           const rw::kinematics::State& state);

	/**
	 * @brief Destructor
	 */
	virtual ~MinimumClearanceCalculator();

	/**
	 * @copydoc ClearanceCalculator::clearance
	 */
	double clearance(rw::kinematics::State& state);

private:
    rw::proximity::DistanceCalculatorPtr _distancecalculator;
};


 /** @} */
} //end pathoptimization
} //end rwlibs

#endif //#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_HPP
