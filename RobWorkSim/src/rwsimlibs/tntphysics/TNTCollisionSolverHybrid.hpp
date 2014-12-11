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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERHYBRID_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERHYBRID_HPP_

/**
 * @file TNTCollisionSolverHybrid.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolverHybrid
 */

#include "TNTCollisionSolver.hpp"

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A collision solver that handles collisions as sequential as much as possible, but reverts to
 * simultaneous handling if sequential handling is not possible.
 *
 * This is similar to the chain solver, but without the requirement that the impulses propagate in a chain.
 *
 * Note that simultaneous handling is not considered correct when multiple objects are solved for simultaneously.
 * However it is often the case in practice that the ability to revert to simultaneous handling gives
 * more robust simulation.
 */
class TNTCollisionSolverHybrid: public rwsimlibs::tntphysics::TNTCollisionSolver {
public:
    //! @brief Empty constructor.
	TNTCollisionSolverHybrid();

    //! @brief Destructor.
	virtual ~TNTCollisionSolverHybrid();

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
	 * This implementation uses TNTCollisionSolverSimultaneous to solve for small simultaneous components of bodies.
	 * For documentation of the properties added by this class, see TNTCollisionSolverSimultaneous::addDefaultProperties .
	 *
	 * This class uses the following properties:
	 *
	 *  Property Name                                         | Type   | Default value | Description
	 *  ----------------------------------------------------- | ------ | ------------- | -----------
	 *  TNTCollisionSolverPropagateThresholdContact           | double | \f$10^{-4}\f$ | Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).
	 *  TNTCollisionSolverPropagateThresholdConstraintLinear  | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).
	 *  TNTCollisionSolverPropagateThresholdConstraintAngular | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).
	 *  TNTCollisionSolverMaxIterations                       | int    |     1000      | If impulses are still propagating after this number of iterations, an exception is thrown.
	 *  -                                                     | -      | -             | TNTCollisionSolverSimultaneous::addDefaultProperties defines more properties used by this solver.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	virtual void handleComponent(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& component,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap) const;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVERHYBRID_HPP_ */
