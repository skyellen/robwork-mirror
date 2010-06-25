/*
 * SequintialImpulseSolver.hpp
 *
 *  Created on: 21-10-2008
 *      Author: jimali
 */

#ifndef SEQUINTIALIMPULSESOLVER_HPP_
#define SEQUINTIALIMPULSESOLVER_HPP_

#include "ConstraintSolver.hpp"

namespace rwsim {
namespace simulator {

class SequintialImpulseSolver: public ConstraintSolver {
public:

    SequintialImpulseSolver(){};

    virtual ~SequintialImpulseSolver(){};

    /**
     * @brief solves the constraints forces of a group of constraints
     * @return
     */
    bool solveGroup( CEdgeGroup& group,
                     SolverInfo& info,
                     rw::kinematics::State& state);


};

}
}

#endif /* SEQUINTIALIMPULSESOLVER_HPP_ */
