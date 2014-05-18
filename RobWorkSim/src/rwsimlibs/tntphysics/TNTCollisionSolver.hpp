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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVER_HPP_

/**
 * @file TNTCollisionSolver.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTCollisionSolver
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Wrench6D.hpp>
#include <vector>

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBodyConstraintManager;
class TNTContact;
class TNTIslandState;
class TNTMaterialMap;
class TNTRigidBody;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Interface for collision solvers that handle bouncing contacts.
 */
class TNTCollisionSolver {
public:
    //! Smart pointer type of TNTCollisionSolver
    typedef rw::common::Ptr<const TNTCollisionSolver> Ptr;

    //! @brief Empty constructor.
	TNTCollisionSolver();

    //! @brief Destructor.
	virtual ~TNTCollisionSolver();

	/**
	 * @brief Find and apply impulses with origin in given contacts.
	 *
	 * Resolving the impulses will in principle involve the complete graph of constraint-connected
	 * bodies. This information is provided by the TNTBodyConstraintManager. The state structures
	 * makes it possible to get the positions and velocities of bodies and constraints, and allows
	 * storing new velocities after they have been changed instantaneously by the applied impulses.
	 *
	 * Impulses will origin in the contacts given as input to the function. These contacts are
	 * expected to have relative velocities that make them penetrating. Restitution models are used
	 * to relate the incoming and outgoing relative velocities of the contacts.
	 *
	 * If there are non-contact constraints between the two objects, impulses are applied in these
	 * constraints to make sure that the relative velocity stays zero.
	 *
	 * There can be contacts between the two objects besides the penetrating contacts given as input.
	 * Due to the impulse applied by these penetrating contacts (and possibly constraint impulses),
	 * the existing contacts might become either leaving or penetrating. An impulse will be applied
	 * at these existing contacts if they become penetrating, to make sure that they do not penetrate.
	 *
	 * @param contacts [in] the list of contacts to apply impulses for.
	 * @param bc [in] the current graph of bodies connected by constraints (including the contacts
	 * given as arguments).
	 * @param map [in] the material map to use to resolve restitution models.
	 * @param tntstate [in/out] the current configuration of the system, where new velocities will
	 * be stored.
	 * @param rwstate [in] the current state.
	 */
	virtual void applyImpulses(
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* map,
			TNTIslandState& tntstate,
			const rw::kinematics::State& rwstate) const = 0;

	/**
	 * @brief Apply an impulse at given position to a body.
	 *
	 * This is mostly a convenience function for the TNTCollisionSolver implementations, as
	 * impulses will typically be applied the same way once they have been found.
	 *
	 * @param impulse [in] the impulse to apply given in world coordinates.
	 * @param position [in] the position to apply the impulse at in world coordinates.
	 * @param body [in] the body to apply the impulse for.
	 * @param tntstate [in/out] the current configuration of the system, where the velocity of the given body will be updated.
	 */
	static void applyImpulse(
			const rw::math::Wrench6D<>& impulse,
			const rw::math::Vector3D<>& position,
			const TNTRigidBody& body,
			TNTIslandState& tntstate);

	/**
	 * @brief Apply multiple impulses to a body.
	 *
	 * This is mostly a convenience function for the TNTCollisionSolver implementations, as
	 * impulses will typically be applied the same way once they have been found.
	 *
	 * @param impulses [in] the impulses to apply given in world coordinates.
	 * @param positions [in] the corresponding positions of where to apply the impulses in world coordinates.
	 * @param body [in] the body to apply the impulses for.
	 * @param tntstate [in/out] the current configuration of the system, where the velocity of the given body will be updated.
	 */
	static void applyImpulses(
			const std::vector<rw::math::Wrench6D<> >& impulses,
			const std::vector<rw::math::Vector3D<> >& positions,
			const TNTRigidBody& body,
			TNTIslandState& tntstate);

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsimlibs::tntphysics::TNTCollisionSolver::Factory,rwsimlibs::tntphysics::TNTCollisionSolver,rwsimlibs.tntphysics.TNTCollisionSolver}
	 */

	/**
	 * @brief A factory for a TNTCollisionSolver. This factory also defines an ExtensionPoint.
	 *
	 * By default the factory provides the following TNTCollisionSolver type:
	 *  - Single (Solver for single collisions) - TNTCollisionSolverSingle
	 */
	class Factory: public rw::common::ExtensionPoint<TNTCollisionSolver> {
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
		 * @return a pointer to a new TNTCollisionSolver.
		 */
		static TNTCollisionSolver::Ptr makeSolver(const std::string& method);

	private:
		Factory();
	};
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTCOLLISIONSOLVER_HPP_ */
