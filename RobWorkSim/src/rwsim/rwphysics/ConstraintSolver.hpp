#ifndef CONSTRAINTSOLVER_HPP_
#define CONSTRAINTSOLVER_HPP_

#include "ConstraintEdge.hpp"
#include <vector>

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {

	//! @brief a group of constraint edges
	typedef std::vector<ConstraintEdge*> CEdgeGroup;

	/**
	 * @brief parameters for the constraint solver
	 */
	struct SolverInfo {
	public:
		SolverInfo(double timestep):
			dt(timestep){}
		double dt;
	};

	/**
	 * @brief Abstraction over a constraint solver for dynamics simulations.
	 *
	 * The constraint solver changes forces, velocities and/or position to correct
	 * constraints between objects in a scene. Most commonly contact constraints
	 * and especially multiple coupled contact constraints are handled.
	 *
	 */
	class ConstraintSolver {

	protected:
		//! @brief constructor
		ConstraintSolver(){};

	public:

		//! @brief destructor
		virtual ~ConstraintSolver(){};

		/**
		 * @brief solves the constraints forces of multiple groups of constraints
		 * @return true if all constraints where solved satisfactory
		 */
		virtual bool solve( std::vector<CEdgeGroup>& groups,
							SolverInfo& info,
							rw::kinematics::State& state);

		/**
		 * @brief solves the constraints forces of a group of constraints
		 * @return true if all constraints where solved satisfactory
		 */
		virtual bool solveGroup( CEdgeGroup& group,
								 SolverInfo& info,
								 rw::kinematics::State& state) = 0;

	};

}}

#endif /*CONSTRAINTSOLVER_HPP_*/
