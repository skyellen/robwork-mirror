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

#ifndef RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSIMULTANEOUS_HPP_
#define RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSIMULTANEOUS_HPP_

/**
 * @file RWPECollisionSolverSimultaneous.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPECollisionSolverSimultaneous
 */

#include "RWPECollisionSolver.hpp"

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEConstraint;
class RWPERestitutionModel;

//! @addtogroup rwsimlibs_rwpe

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
class RWPECollisionSolverSimultaneous: public rwsimlibs::rwpe::RWPECollisionSolver {
public:
    //! @brief Empty constructor.
	RWPECollisionSolverSimultaneous();

    //! @brief Destructor.
	virtual ~RWPECollisionSolverSimultaneous();

	//! @copydoc RWPECollisionSolver::doCollisions
	virtual void doCollisions(
			const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph& bc,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL,
			rw::common::Ptr<rw::common::ThreadTask> task = NULL) const;

	/**
	 * @copybrief RWPECollisionSolver::addDefaultProperties
	 *
	 * This implementation uses the following properties:
	 *
	 *  Property Name                            | Type   | Default value | Description
	 *  ---------------------------------------- | ------ | ------------- | -----------
	 *  RWPECollisionSolverResolverTolerance      | double | \f$10^{-6}\f$ | Resolver will activate contacts with colliding velocity greater than this, and deactivate contacts that has leaving velocity greater than this (in m/s).
	 *  RWPECollisionSolverSingularValuePrecision | double | \f$10^{-6}\f$ | Precision of SVD - see rw::math::LinearAlgebra::pseudoInverse(const Eigen::MatrixXd&, double) .
	 *  RWPECollisionSolverMaxContacts            | int    | \f$24\f$      | Maximum number of simultaneous contacts before throwing exception (only used if solve iterative i disabled).
	 *  RWPECollisionSolverIterations             | int    | \f$2500\f$    | Maximum number of iterations in iterative solver (set to zero to disable iterative solver).
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

	/**
	 * @brief Solve for a subcomponent, while resolving the contacts as enabled/disabled.
	 * @param component [in] a list of the bodies to solve for in the component.
	 * @param constraints [in] a list of contacts and constraints to solve for in the component.
	 * @param map [in] a map to retrieve restitution models from.
	 * @param islandState [in/out] the state to update with new velocities.
	 * @param rwstate [in] the current state.
	 * @param pmap [in] properties as desribed in #addDefaultProperties.
	 * @param log [in] log utility to log to (optional).
	 */
	static void resolveContacts(
			const std::list<const RWPEBodyDynamic*>& component,
			const std::list<const RWPEConstraint*>& constraints,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL);

private:
	static void addDefaultPropertiesInternal(rw::common::PropertyMap& map);

	static void handleComponent(
			//const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph& bc,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL);

	static Eigen::VectorXd solve(
			const std::vector<const RWPEContact*>& contacts,
			const std::vector<const RWPEConstraint*>& constraints,
			const RWPEMaterialMap* map,
			const RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			double precision,
			bool iterative,
			unsigned int iterations,
			class RWPELogUtil* log = NULL);

	static Eigen::VectorXd iterativeSVD(
			const Eigen::MatrixXd& A,
			const Eigen::VectorXd& b,
			Eigen::MatrixXd::Index constraintDim,
			unsigned int iterations,
			double svdPrecision,
			double eps,
			class RWPELogUtil* log = NULL);

	static void applySolution(
			const Eigen::VectorXd& solution,
			const std::list<const RWPEBodyDynamic*>& component,
			const std::vector<const RWPEContact*>& contacts,
			const std::vector<const RWPEConstraint*>& constraints,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const RWPEConstraint* constraintA,
			const RWPEConstraint* constraintB,
			const RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const RWPEConstraint* constraintA,
			const RWPEContact* contactB,
			const RWPERestitutionModel& restitutionModel,
			const RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const RWPEContact* contactA,
			const RWPEConstraint* constraintB,
			const RWPERestitutionModel& restitutionModel,
			const RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate);

	static Eigen::MatrixXd getBlock(
			const RWPEContact* contactA,
			const RWPEContact* contactB,
			const RWPERestitutionModel& restitutionModelA,
			const RWPERestitutionModel& restitutionModelB,
			const RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate);
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSIMULTANEOUS_HPP_ */
