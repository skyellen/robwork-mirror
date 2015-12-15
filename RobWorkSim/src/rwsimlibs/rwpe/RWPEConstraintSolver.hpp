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

#ifndef RWSIMLIBS_RWPE_RWPESOLVER_HPP_
#define RWSIMLIBS_RWPE_RWPESOLVER_HPP_

/**
 * @file RWPEConstraintSolver.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEConstraintSolver
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/math/Vector3D.hpp>

#include <Eigen/Eigen>

// Forward declarations
namespace rw { namespace kinematics { class State; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBodyConstraintGraph;
class RWPEIslandState;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Sets up the equation system for constraint forces and solves it.
 */
class RWPEConstraintSolver {
public:
	//! @brief Destructor.
	virtual ~RWPEConstraintSolver() {};

	/**
	 * @brief Create a new solver.
     * @param manager [in] a pointer to the graph of bodies and constraints to use.
     * @param gravity [in] the gravity in world coordinates.
	 * @return a pointer to a new RWPEConstraintSolver - owned by the caller.
	 */
	virtual const RWPEConstraintSolver* createSolver(const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity) const = 0;

	/**
	 * @brief Solve the constraint forces for the constraints in the system.
	 * @param h [in] the timestep to solve for.
	 * @param discontinuity [in] true if the integration scheme should be different because of a discontinuity.
	 * @param rwstate [in] the current state.
	 * @param islandState0 [in] the current RWPEIslandState at beginning of time step.
	 * @param islandStateH [in] the RWPEIslandState predicted at end of time step.
	 * @param pmap [in] properties to use - see #addDefaultProperties for details.
	 * @param log [in] (optional) do logging.
	 * @return a raw vector with the solution.
	 */
	virtual Eigen::VectorXd solve(double h, bool discontinuity, const rw::kinematics::State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH, const rw::common::PropertyMap& pmap, class RWPELogUtil* log = NULL) const;

	/**
	 * @brief Solve the dynamics.
	 *
	 * The dynamics problem is often written as a complementarity problem \f${\bf A} {\bf f} \geq {\bf b}\f$ where \f${\bf f} \geq {\bf 0}\f$.
	 * This is an equation for the contact and constraint velocities given the interaction forces and torques, \f${\bf f}\f$.
	 *
	 * @param A [in] the system matrix on the left hand side of the equation system. This matrix must be symmetric and Positive Semi-definite.
	 * @param b [in] the right hand side of the equation system.
	 * @param constraintDim [in] the number of constraints (assumed to be to first equations in system)
	 * @param pmap [in] properties to use - see #addDefaultProperties for details.
	 * @param log [in] (optional) do logging.
	 * @return the vector \f${\bf f}\f$.
	 */
	virtual Eigen::VectorXd solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::MatrixXd::Index constraintDim, const rw::common::PropertyMap& pmap, class RWPELogUtil* log = NULL) const = 0;

	/**
	 * @brief Save the forces and torques obtained with solve() to the state.
	 * @param solution [in] the force/torque vector.
	 * @param state [in/out] the state.
	 * @param log [in] (optional) do logging.
	 */
	virtual void saveSolution(const Eigen::VectorXd& solution, RWPEIslandState &state, class RWPELogUtil* log = NULL) const;

	/**
	 * @brief Get the body-constraint manager used by the solver.
	 * @return a pointer to a constant RWPEBodyConstraintGraph - NOT owned by caller.
	 */
	virtual const RWPEBodyConstraintGraph* getManager() const;

	/**
	 * @brief Get gravity used by the solver.
	 * @return a reference to the gravity.
	 */
	virtual const rw::math::Vector3D<>& getGravity() const;

	/**
	 * @brief Add the default properties to the given map.
	 *
	 * Please look at the documentation for the specific implementations of this function to get information about
	 * the required properties for these implementations.
	 *
	 *  Property Name              | Type   | Default value | Description
	 *  -------------------------- | ------ | ------------- | -----------
	 *  RWPEConstraintSolverDebug  | int    | 0             | Enable or disable debugging (really slow).
	 *
	 * @param map [in/out] the map to add the default properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPEConstraintSolver::Factory,rwsimlibs::rwpe::RWPEConstraintSolver,rwsimlibs.rwpe.RWPEConstraintSolver}
	 */

	/**
	 * @brief A factory for a RWPEConstraintSolver. This factory also defines an
	 * extension point for RWPEConstraintSolver.
	 *
	 * By default the factory provides the following RWPEConstraintSolver types:
	 *  - SVD - RWPEConstraintSolverSVD
	 */
    class Factory: public rw::common::ExtensionPoint<RWPEConstraintSolver> {
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
    	 * @return a pointer to a new RWPEConstraintSolver - the pointer is owned by the caller.
    	 */
    	static const RWPEConstraintSolver* makeSolver(const std::string& solverType, const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity);

    private:
        Factory();
    };

protected:
    /**
     * @brief Construct new solver.
     * @param manager [in] the manager.
     * @param gravity [in] the gravity.
     */
	RWPEConstraintSolver(const RWPEBodyConstraintGraph* manager, const rw::math::Vector3D<double> &gravity);

	//! @brief The bodies and constraints.
	const RWPEBodyConstraintGraph* const _manager;

	//! @brief The gravity used by the solver.
	const rw::math::Vector3D<> _gravity;

private:
	void getMatrices(Eigen::MatrixXd& lhs, Eigen::VectorXd& rhs, Eigen::MatrixXd::Index& constraintDim, double h, bool discontinuity, const rw::kinematics::State &rwstate, const RWPEIslandState &islandState0, const RWPEIslandState &islandStateH, bool debug, class RWPELogUtil* log) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPESOLVER_HPP_ */
