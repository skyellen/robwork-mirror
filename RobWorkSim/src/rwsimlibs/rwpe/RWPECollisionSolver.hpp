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

#ifndef RWSIMLIBS_RWPE_RWPECOLLISIONSOLVER_HPP_
#define RWSIMLIBS_RWPE_RWPECOLLISIONSOLVER_HPP_

/**
 * @file RWPECollisionSolver.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPECollisionSolver
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Wrench6D.hpp>
#include <vector>

// Forward declarations
namespace rw { namespace common { class ThreadTask; } }

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBodyConstraintGraph;
class RWPEContact;
class RWPEIslandState;
class RWPEMaterialMap;
class RWPEBodyDynamic;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Interface for collision solvers that handle bouncing contacts.
 *
 * Different paradigms exist for collision handling, and this interface defines a high-level
 * abstraction for different solvers that can be used to resolve collisions. The Factory
 * defines a default set of collisions solvers, and allows extension with custom solvers if
 * required.
 *
 * Please see the Factory class for an overview of available solvers.
 */
class RWPECollisionSolver {
public:
    //! Smart pointer type of RWPECollisionSolver
    typedef rw::common::Ptr<const RWPECollisionSolver> Ptr;

    //! @brief Destructor.
	virtual ~RWPECollisionSolver();

	/**
	 * @brief Find and apply impulses with origin in given contacts.
	 *
	 * Resolving the impulses involves the complete graph of contact/constraint-connected
	 * bodies. This information is provided by the RWPEBodyConstraintGraph. The state structures
	 * makes it possible to get the positions and velocities of bodies and constraints, and allows
	 * storing new velocities after they have been changed instantaneously by the applied impulses.
	 *
	 * Impulses will origin in the contacts given as input to the function. These contacts are
	 * expected to have relative velocities that make them colliding. Restitution models are used
	 * to relate the incoming and outgoing relative velocities of the contacts. These will be
	 * retrieved from the RWPEMaterialMap.
	 *
	 * If there are non-contact constraints between the two objects, impulses are applied in these
	 * constraints to make sure that the relative velocity stays zero.
	 *
	 * There can be contacts between the two objects besides the colliding contacts given as input.
	 * Due to the impulse applied by the colliding contacts (and possibly constraint impulses),
	 * the existing contacts might become either leaving or penetrating. An impulse will be applied
	 * at these existing contacts if they become penetrating, to make sure that they do not penetrate.
	 *
	 * @param contacts [in] the list of initiating colliding contacts.
	 * @param bc [in] the current graph of bodies connected by constraints (the colliding contact
	 * are expected to be included in this graph as well).
	 * @param map [in] the material map to use to resolve restitution models.
	 * @param islandState [in/out] the current configuration of the system, where new velocities will
	 * be stored.
	 * @param rwstate [in] the current state.
	 * @param pmap [in] properties to use - see #addDefaultProperties for details.
	 * @param log [in] (optional) do logging.
	 * @param task [in] (optional) task to add work to.
	 */
	virtual void doCollisions(
			const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph& bc,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL,
			rw::common::Ptr<rw::common::ThreadTask> task = NULL) const = 0;

	/**
	 * @brief Apply an impulse at given position to a body.
	 *
	 * This is mostly a convenience function for the RWPECollisionSolver implementations, as
	 * impulses will typically be applied the same way once they have been found.
	 *
	 * @param impulse [in] the impulse to apply given in world coordinates.
	 * @param position [in] the position to apply the impulse at in world coordinates.
	 * @param body [in] the body to apply the impulse for.
	 * @param islandState [in/out] the current configuration of the system, where the velocity of the given body will be updated.
	 */
	static void applyImpulse(
			const rw::math::Wrench6D<>& impulse,
			const rw::math::Vector3D<>& position,
			const RWPEBodyDynamic& body,
			RWPEIslandState& islandState);

	/**
	 * @brief Apply multiple impulses to a body.
	 *
	 * This is mostly a convenience function for the RWPECollisionSolver implementations, as
	 * impulses will typically be applied the same way once they have been found.
	 *
	 * @param impulses [in] the impulses to apply given in world coordinates.
	 * @param positions [in] the corresponding positions of where to apply the impulses in world coordinates.
	 * @param body [in] the body to apply the impulses for.
	 * @param islandState [in/out] the current configuration of the system, where the velocity of the given body will be updated.
	 */
	static void applyImpulses(
			const std::vector<rw::math::Wrench6D<> >& impulses,
			const std::vector<rw::math::Vector3D<> >& positions,
			const RWPEBodyDynamic& body,
			RWPEIslandState& islandState);

	/**
	 * @brief Add the default properties to the given map.
	 *
	 * Please look at the documentation for the specific implementations of this function to get information about
	 * the required properties for these implementations.
	 *
	 * @param map [in/out] the map to add the default properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

	/**
	 * @brief Get a new PropertyMap with default properties.
	 * @return PropertyMap with all default properties.
	 */
	rw::common::PropertyMap getDefaultMap() const;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::rwpe::RWPECollisionSolver::Factory,rwsimlibs::rwpe::RWPECollisionSolver,rwsimlibs.rwpe.RWPECollisionSolver}
	 */

	/**
	 * @brief A factory for a RWPECollisionSolver. This factory also defines an ExtensionPoint.
	 *
	 * By default the factory provides the following RWPECollisionSolver types:
	 *  - None (Throws exception if a collision occur)
	 *  - Single (Solver for one or more collisions between a single pair of bodies) - RWPECollisionSolverSingle
	 *  - Chain (Sequential solver that applies the single solver iteratively) - RWPECollisionSolverChain
	 *  - Simultaneous (Solver for simultaneous handling) - RWPECollisionSolverSimultaneous
	 *  - Hybrid (A hybrid between sequential and simultaneous handling that applies the simultaneous solver iteratively) - RWPECollisionSolverHybrid
	 *
	 *  A good introduction to different collision handling methods can be found at:
	 *  http://www.myphysicslab.com/Collision-methods.html
	 */
	class Factory: public rw::common::ExtensionPoint<RWPECollisionSolver> {
	public:
		/**
		 * @brief Get the available collision solvers.
		 * @return a vector of identifiers for solver methods.
		 */
		static std::vector<std::string> getSolvers();

		/**
		 * @brief Check if collision solver is available.
		 * @param method [in] the name of the collision solver.
		 * @return true if available, false otherwise.
		 */
		static bool hasSolver(const std::string& method);

		/**
		 * @brief Create a new collision solver.
		 * @param method [in] the name of the collision solver.
		 * @return a pointer to a new RWPECollisionSolver.
		 */
		static RWPECollisionSolver::Ptr makeSolver(const std::string& method);

	private:
		Factory();
	};

protected:
	//! @brief Constructor.
	RWPECollisionSolver();

private:
	class RWPECollisionSolverNone;
	const rw::common::PropertyMap _emtpyMap;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECOLLISIONSOLVER_HPP_ */
