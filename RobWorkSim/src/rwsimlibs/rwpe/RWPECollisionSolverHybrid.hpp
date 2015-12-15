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

#ifndef RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERHYBRID_HPP_
#define RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERHYBRID_HPP_

/**
 * @file RWPECollisionSolverHybrid.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPECollisionSolverHybrid
 */

#include "../rwpe/RWPECollisionSolver.hpp"

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

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
class RWPECollisionSolverHybrid: public rwsimlibs::rwpe::RWPECollisionSolver {
public:
    //! @brief Empty constructor.
	RWPECollisionSolverHybrid();

    //! @brief Destructor.
	virtual ~RWPECollisionSolverHybrid();

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
	 * This implementation uses RWPECollisionSolverSimultaneous to solve for small simultaneous components of bodies.
	 * For documentation of the properties added by this class, see RWPECollisionSolverSimultaneous::addDefaultProperties .
	 *
	 * This class uses the following properties:
	 *
	 *  Property Name                                         | Type   | Default value | Description
	 *  ----------------------------------------------------- | ------ | ------------- | -----------
	 *  RWPECollisionSolverPropagateThresholdContact           | double | \f$10^{-4}\f$ | Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).
	 *  RWPECollisionSolverPropagateThresholdConstraintLinear  | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).
	 *  RWPECollisionSolverPropagateThresholdConstraintAngular | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).
	 *  RWPECollisionSolverMaxIterations                       | int    |     1000      | If impulses are still propagating after this number of iterations, an exception is thrown.
	 *  -                                                     | -      | -             | RWPECollisionSolverSimultaneous::addDefaultProperties defines more properties used by this solver.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	virtual void handleComponent(
			const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph& component,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERHYBRID_HPP_ */
