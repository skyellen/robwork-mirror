/*
 * SimStateConstraint.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef SIMSTATECONSTRAINT_HPP_
#define SIMSTATECONSTRAINT_HPP_

#include <rw/common/Ptr.hpp>

class SimStateConstraint {
public:
    virtual bool isSatisfied(const rw::kinematics::State &state, Simulator *sim) = 0;
};

typedef rw::common::Ptr<SimStateConstraint> SimStateConstraintPtr;

#endif /* SIMSTATECONSTRAINT_HPP_ */
