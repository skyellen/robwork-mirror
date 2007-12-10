#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP
#define RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP

#include <rw/kinematics/State.hpp>

namespace rwlibs {
namespace pathoptimization{

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

} //end namespace pathoptimization
} //end namespace rwlibs

#endif //#ifndef RWLIBS_PATHOPTIMIZATION_CLEARANCECALCULATOR_HPP
