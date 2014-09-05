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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSINGLE_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSINGLE_HPP_

/**
 * @file TNTCollisionSolverSingle.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolverSingle
 */

#include "TNTCollisionSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBody;
class TNTConstraint;
class TNTRestitutionModel;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The most basic implementation of a TNTCollisionSolver.
 *
 * The solver is only able to resolve a single collision at a time, and will throw
 * an exception if there are multiple contacts/constraints at the same time.
 *
 * A rigid body that has a new contact that should be resolved with this method,
 * should not have any other contacts or constraints defined. For kinematic objects
 * there can be multiple contacts and constraints, as long as they are not between
 * the same two objects being resolved with this method.
 *
 * If this strategy is used as a part of a larger strategy, the exceptions can be
 * disabled with the setCheckNoChain() function.
 */
class TNTCollisionSolverSingle: public rwsimlibs::tntphysics::TNTCollisionSolver {
public:
    //! @brief Empty constructor.
	TNTCollisionSolverSingle();

    //! @brief Destructor.
	virtual ~TNTCollisionSolverSingle();

	//! @copydoc TNTCollisionSolver::applyImpulses
	virtual void applyImpulses(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	/**
	 * @brief Find and apply impulses such that constraints are satisfied between two bodies.
	 *
	 * This function is provided as a utility function for more complex collision solvers.
	 * If there are no penetrating contacts between two bodies, this function can be used to solve
	 * for constraint impulses. If the non-penetrating contacts becomes penetrating when applying
	 * constraint impulses, impulses are applied for these contacts as well to keep them from
	 * penetrating.
	 *
	 * @param constraints [in] the list of constraints to apply impulses for.
	 * @param bc [in] the current graph of bodies connected by constraints (including the contacts
	 * given as arguments).
	 * @param map [in] the material map to use to resolve restitution models.
	 * @param tntstate [in/out] the current configuration of the system, where new velocities will
	 * be stored.
	 * @param rwstate [in] the current state.
	 */
	virtual void applyImpulses(
			const TNTBody* bodyA,
			const TNTBody* bodyB,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	/**
	 * @brief Get status of the optional check for impulse chains.
	 * @return true if enabled (default if not actively disabled)
	 */
	bool doCheckNoChain() const;

	/**
	 * @brief Enable or disable check for impulse chains.
	 *
	 * Check is enabled by default, meaning an exception will be thrown if a
	 * chain is encountered.
	 *
	 * Disabling this check makes sense if the solver is used as part of a more
	 * complex strategy for solving impulse chains.
	 *
	 * @param enable [in] true if check should be enabled (default), or false to disable check.
	 */
	void setCheckNoChain(bool enable);

private:
	void resolveContacts(
			const TNTBody* parent,
			const TNTBody* child,
			const std::vector<const TNTContact*>& contacts,
			const std::list<const TNTConstraint*>& constraints,
			const TNTRestitutionModel& restitutionModel,
			const TNTBodyConstraintManager& bc,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	Eigen::VectorXd solve(
			const TNTBody* parent,
			const TNTBody* child,
			const std::vector<const TNTContact*>& contacts,
			const std::vector<const TNTConstraint*>& constraints,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	void applySolution(
			const Eigen::VectorXd& solution,
			const TNTBody* parent,
			const TNTBody* child,
			const std::vector<const TNTContact*>& contacts,
			const std::vector<const TNTConstraint*>& constraints,
			const TNTRestitutionModel& restitutionModel,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	Eigen::MatrixXd getBlock(
			const TNTConstraint* constraintA,
			const TNTConstraint* constraintB,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	Eigen::MatrixXd getBlock(
			const TNTConstraint* constraintA,
			const TNTContact* constraintB,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	Eigen::MatrixXd getBlock(
			const TNTContact* constraintA,
			const TNTContact* constraintB,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	Eigen::MatrixXd getBlock(
			const TNTContact* constraintA,
			const TNTConstraint* constraintB,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const;

	bool _checkNoChain;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSINGLE_HPP_ */
