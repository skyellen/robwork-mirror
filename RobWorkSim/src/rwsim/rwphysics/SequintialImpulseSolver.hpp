/*
 * SequintialImpulseSolver.hpp
 *
 *  Created on: 21-10-2008
 *      Author: jimali
 */

#ifndef RWSIM_SIMULATION_SEQUINTIALIMPULSESOLVER_HPP_
#define RWSIM_SIMULATION_SEQUINTIALIMPULSESOLVER_HPP_

#include "ConstraintSolver.hpp"

namespace rwsim {
namespace simulator {

	/**
	 * @brief a constraint solver that use the sequential impulse algorithm for solving constraints.
	 * this algorithm is supposed to be equal to the Projected Gauss Seidel (PGS) algorithm.
	 */
	class SequintialImpulseSolver: public ConstraintSolver {
	public:

		//! @brief constructor
		SequintialImpulseSolver(){};

		//! @brief destructor
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
