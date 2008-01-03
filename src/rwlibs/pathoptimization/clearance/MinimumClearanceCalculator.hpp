#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_H
#define RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_H

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
     * @brief Constructs a MinimumClearanceCalculator using the \a DistanceCalculator provided.
     * 
     * Use this constructor to use an already existing DistanceCalculator
     * 
     * @param distancecalculator [in] The distance calculator to use 
     */
	MinimumClearanceCalculator(boost::shared_ptr<rw::proximity::DistanceCalculator> distancecalculator);
	
	/**
	 * @brief Constructs a MinimumClearanceCalculator for a workcell
	 * @param workcell [in] WorkCell for which to calculate the minimum clearance
	 */
	MinimumClearanceCalculator(rw::models::WorkCell* workcell, const rw::kinematics::State& _state);
	
	/**
	 * @brief Destructor
	 */
	virtual ~MinimumClearanceCalculator();
	
	/**
	 * @copydoc ClearanceCalculator::clearance
	 */
	double clearance(rw::kinematics::State& state);
	
private:
    boost::shared_ptr<rw::proximity::DistanceCalculator> _distancecalculator;
};


 /** @} */
} //end pathoptimization
} //end rwlibs

#endif //#ifndef RWLIBS_PATHOPTIMIZATION_MINIMUMCLEARANCECALCULATOR_H
