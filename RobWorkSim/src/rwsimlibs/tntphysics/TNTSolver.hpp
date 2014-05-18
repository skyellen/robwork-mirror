/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTSOLVER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTSOLVER_HPP_

/**
 * @file TNTSolver.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTSolver
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/math/Vector3D.hpp>

#include <Eigen/Eigen>

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBodyConstraintManager;
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Sets up the equation system for constraint forces and solves it.
 */
class TNTSolver {
public:
	//! @brief Destructor.
	virtual ~TNTSolver() {};

	/**
	 * @brief Create a new solver.
     * @param manager [in] a pointer to the graph of bodies and constraints to use.
     * @param gravity [in] the gravity in world coordinates.
	 * @return a pointer to a new TNTSolver - owned by the caller.
	 */
	virtual const TNTSolver* createSolver(const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity) const = 0;

	/**
	 * @brief Solve the constraint forces for the constraints in the system.
	 * @param h [in] the timestep to solve for.
	 * @param rwstate [in] the current state.
	 * @param tntstate [in] the current TNTIslandState.
	 * @return a raw vector with the solution.
	 */
	virtual Eigen::VectorXd solve(double h, const rw::kinematics::State &rwstate, const TNTIslandState &tntstate) const = 0;

	/**
	 * @brief Save the forces and torques obtained with solve() to the state.
	 * @param solution [in] the force/torque vector.
	 * @param state [in/out] the state.
	 */
	virtual void saveSolution(const Eigen::VectorXd& solution, TNTIslandState &state) const = 0;

	/**
	 * @brief Get the body-constraint manager used by the solver.
	 * @return a pointer to a constant TNTBodyConstraintManager - NOT owned by caller.
	 */
	virtual const TNTBodyConstraintManager* getManager() const = 0;

	/**
	 * @brief Get gravity used by the solver.
	 * @return a reference to the gravity.
	 */
	virtual const rw::math::Vector3D<>& getGravity() const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::tntphysics::TNTSolver::Factory,rwsimlibs::tntphysics::TNTSolver,rwsimlibs.tntphysics.TNTSolver}
	 */

	/**
	 * @brief A factory for a TNTSolver. This factory also defines an
	 * extension point for TNTSolver.
	 *
	 * By default the factory provides the following TNTSolver types:
	 *  - SVD - TNTSolverSVD
	 */
    class Factory: public rw::common::ExtensionPoint<TNTSolver> {
    public:
    	/**
    	 * @brief Get the available solver types.
    	 * @return a vector of identifiers for solvers.
    	 */
    	static std::vector<std::string> getSolvers();

    	/**
    	 * @brief Check if solver type is available.
    	 * @param solverType [in] the name of the solver.
    	 * @return true if available, false otherwise.
    	 */
    	static bool hasSolver(const std::string& solverType);

    	/**
    	 * @brief Create a new solver.
    	 * @param solverType [in] the name of the solver.
    	 * @param manager [in] a pointer to the graph of bodies and constraints to use.
    	 * @param gravity [in] the gravity in world coordinates.
    	 * @return a pointer to a new TNTSolver - the pointer is owned by the caller.
    	 */
    	static const TNTSolver* makeSolver(const std::string& solverType, const TNTBodyConstraintManager* manager, const rw::math::Vector3D<double> &gravity);

    private:
        Factory();
    };

protected:
	TNTSolver() {};
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTSOLVER_HPP_ */
