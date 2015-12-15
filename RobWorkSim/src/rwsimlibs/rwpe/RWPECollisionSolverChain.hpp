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

#ifndef RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERCHAIN_HPP_
#define RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERCHAIN_HPP_

/**
 * @file RWPECollisionSolverChain.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPECollisionSolverChain
 */

#include "../rwpe/RWPECollisionSolver.hpp"

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPECollisionSolverSingle;
class RWPEBody;
class RWPEBodyFixed;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief A collision solver that can handle impulse chains.
 *
 * The solver uses the RWPECollisionSolverSingle to solve for individual pairs of objects, and
 * solves iteratively for the objects in the chain as the impulses propagate back and forth.
 *
 * An exception will be thrown if the bodies are connected in a circular or tree type structure.
 *
 * The impulse must either origin from only one contact pair in the chain, or the contact pairs must not
 * share objects. Otherwise an exception will be thrown as it is not possible to simultaneously change the
 * velocity of an object if it is involved in collisions with more than one object.
 */
class RWPECollisionSolverChain: public rwsimlibs::rwpe::RWPECollisionSolver {
public:
    //! @brief Empty constructor.
	RWPECollisionSolverChain();

    //! @brief Destructor.
	virtual ~RWPECollisionSolverChain();

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
	 * This implementation uses RWPECollisionSolverSingle to solve for pairs of bodies.
	 * See documentation for this class for available properties.
	 *
	 *  Property Name                                         | Type   | Default value | Description
	 *  ----------------------------------------------------- | ------ | ------------- | -----------
	 *  RWPECollisionSolverPropagateThresholdContact           | double | \f$10^{-4}\f$ | Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).
	 *  RWPECollisionSolverPropagateThresholdConstraintLinear  | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).
	 *  RWPECollisionSolverPropagateThresholdConstraintAngular | double | \f$10^{-8}\f$ | Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).
	 *  RWPECollisionSolverMaxIterations                       | int    |     1000      | If impulses are still propagating after this number of iterations, an exception is thrown.
	 *  -                                                     | -      | -             | RWPECollisionSolverSingle::addDefaultProperties defines more properties used by this solver.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	struct SharedInfo;
	class DecomposeTask;
	class ChainTask;

	struct Chain {
		std::set<const RWPEBodyFixed*> fixedBodiesBegin;
		std::vector<const RWPEBody*> bodies;
		std::set<const RWPEBodyFixed*> fixedBodiesEnd;
	};

	static Chain constructChain(
			const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph& component,
			const RWPEIslandState& islandState);

	static std::set<std::size_t> getIndices(
		const std::vector<const RWPEContact*>& contacts,
		const Chain& chain);

	RWPECollisionSolverSingle* _solver;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERCHAIN_HPP_ */
