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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSIMULTANEOUS_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSIMULTANEOUS_HPP_

/**
 * @file TNTCollisionSolverSimultaneous.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolverSimultaneous
 */

#include "TNTCollisionSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTConstraint;
class TNTRestitutionModel;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A collision solver that handles all collisions simultaneously.
 *
 * This solver is very robust, but should not be expected to give physically correct results.
 * Use this if realistic collision handling is not important, or as a last resort if
 * other methods fail. Note that this solver is used by many other solvers to solve for smaller
 * subproblems simultaneously.
 *
 * The collision solver includes a resolver that makes sure that contact impulses are never attracting.
 * If a contact becomes attracting, it is deactivated and the system is resolved assuming there is no
 * impulse at this contact. If on the other hand a contact is deactivated and has normal velocity that
 * causes it to be colliding, it is reactivated. See #addDefaultProperties for description of the property
 * controling the tolerance.
 */
class TNTCollisionSolverSimultaneous: public rwsimlibs::tntphysics::TNTCollisionSolver {
public:
    //! @brief Empty constructor.
	TNTCollisionSolverSimultaneous();

    //! @brief Destructor.
	virtual ~TNTCollisionSolverSimultaneous();

	//! @copydoc TNTCollisionSolver::doCollisions
	virtual void doCollisions(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			rw::common::Ptr<rw::common::ThreadTask> task = NULL) const;

	/**
	 * @copybrief TNTCollisionSolver::addDefaultProperties
	 *
	 * This implementation uses the following properties:
	 *
	 *  Property Name                            | Type   | Default value | Description
	 *  ---------------------------------------- | ------ | ------------- | -----------
	 *  TNTCollisionSolverResolverTolerance      | double | \f$10^{-6}\f$ | Resolver will activate contacts with colliding velocity greater than this, and deactivate contacts that has leaving velocity greater than this (in m/s).
	 *  TNTCollisionSolverSingularValuePrecision | double | \f$10^{-6}\f$ | Precision of SVD - see rw::math::LinearAlgebra::pseudoInverse(const Eigen::MatrixXd&, double) .
	 *  TNTCollisionSolverMaxContacts            | int    | \f$24\f$      | Maximum number of simultaneous contacts before throwing exception (only used if solve iterative i disabled).
	 *  TNTCollisionSolverIterations             | int    | \f$200\f$     | Maximum number of iterations in iterative solver (set to zero to disable iterative solver).
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

	/**
	 * @brief Solve for a subcomponent, while resolving the contacts as enabled/disabled.
	 * @param component [in] a list of the bodies to solve for in the component.
	 * @param constraints [in] a list of contacts and constraints to solve for in the component.
	 * @param map [in] a map to retrieve restitution models from.
	 * @param tntstate [in/out] the state to update with new velocities.
	 * @param rwstate [in] the current state.
	 * @param pmap [in] properties as desribed in #addDefaultProperties .
	 */
	static void resolveContacts(
			const std::list<const TNTRigidBody*>& component,
			const std::list<const TNTConstraint*>& constraints,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap);

private:
	static void addDefaultPropertiesInternal(rw::common::PropertyMap& map);

	static void handleComponent(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap);

	static Eigen::VectorXd solve(
			const std::vector<const TNTContact*>& contacts,
			const std::vector<const TNTConstraint*>& constraints,
			const TNTMaterialMap* map,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			double precision,
			bool iterative,
			unsigned int iterations);

	static Eigen::VectorXd iterativeSVD(
			const Eigen::MatrixXd& A,
			const Eigen::VectorXd& b,
			Eigen::MatrixXd::Index constraintDim,
			unsigned int iterations,
			double svdPrecision,
			double eps);

	static void applySolution(
			const Eigen::VectorXd& solution,
			const std::list<const TNTRigidBody*>& component,
			const std::vector<const TNTContact*>& contacts,
			const std::vector<const TNTConstraint*>& constraints,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const TNTConstraint* constraintA,
			const TNTConstraint* constraintB,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const TNTConstraint* constraintA,
			const TNTContact* contactB,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const TNTContact* contactA,
			const TNTConstraint* constraintB,
			const TNTRestitutionModel& restitutionModel,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const TNTContact* contactA,
			const TNTContact* contactB,
			const TNTRestitutionModel& restitutionModelA,
			const TNTRestitutionModel& restitutionModelB,
			const TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate);
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERSIMULTANEOUS_HPP_ */
