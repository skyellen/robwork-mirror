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

#ifndef RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSINGLE_HPP_
#define RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSINGLE_HPP_

/**
 * @file RWPECollisionSolverSingle.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPECollisionSolverSingle
 */

#include "../rwpe/RWPECollisionSolver.hpp"

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBody;
class RWPEBodyFixed;
class RWPEBodyDynamic;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The most simple implementation of a RWPECollisionSolver.
 *
 * The solver is only able to solve collisions between a pair of objects if these
 * objects are not connected by contacts or constraints to any other objects. If
 * more than two objects are in contact, this method will throw an exception.
 *
 * Note that the solver solves for all constraints and contacts between two objects
 * simultaneously. It uses the RWPECollisionSolverSimultaneous solver for this purpose.
 * Please see this class for further description of the method and properties.
 */
class RWPECollisionSolverSingle: public rwsimlibs::rwpe::RWPECollisionSolver {
public:
    //! @brief Empty constructor.
	RWPECollisionSolverSingle();

    //! @brief Destructor.
	virtual ~RWPECollisionSolverSingle();

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
	 * @brief Find and apply impulses such that contacts and constraints are satisfied between two bodies.
	 *
	 * This function is provided as a utility function for more complex collision solvers.
	 * The function does not consider initiating collisions, but solves for all contacts and constraints
	 * between the two given bodies. Collisions can be applied if the two bodies has come in collision due
	 * to collisions with other objects.
	 *
	 * @note At least on of the two bodies must be a RWPEBodyDynamic.
	 *
	 * @param bodyA [in] the first body of the pair to apply impulses for.
	 * @param bodyB [in] the second body of the pair to apply impulses for.
	 * @param bc [in] the current graph of bodies connected by contacts  and constraints.
	 * @param map [in] the material map to use to resolve restitution models.
	 * @param islandState [in/out] the current configuration of the system, where new velocities will
	 * be stored.
	 * @param rwstate [in] the current state.
	 * @param pmap [in] properties to use - see #addDefaultProperties for details.
	 * @param log [in] (optional) add logging info.
	 * @param task [in] (optional) task to add work to.
	 */
	virtual void doCollisions(
			const RWPEBody* bodyA,
			const RWPEBody* bodyB,
			const RWPEBodyConstraintGraph& bc,
			const RWPEMaterialMap* map,
			RWPEIslandState& islandState,
			const rw::kinematics::State& rwstate,
			const rw::common::PropertyMap& pmap,
			class RWPELogUtil* log = NULL,
			rw::common::Ptr<rw::common::ThreadTask> task = NULL) const;

	/**
	 * @brief Find and apply impulses between a rigid body and multiple fixed bodies.
	 *
	 * This function is provided as a utility function for more complex collision solvers.
	 * The function does not consider initiating collisions, but solves for all contacts and constraints
	 * between the two given bodies. Collisions can be applied if the two bodies has come in collision due
	 * to collisions with other objects.
	 *
	 * This function uses the fact that multiple fixed bodies can be considered one static object.
	 *
	 * @param rigidBody [in] the rigid body.
	 * @param fixedBodies [in] a list of fixed bodies that the rigid body is in collision with.
	 * @param bc [in] the current graph of bodies connected by contacts  and constraints.
	 * @param map [in] the material map to use to resolve restitution models.
	 * @param islandState [in/out] the current configuration of the system, where new velocities will
	 * be stored.
	 * @param rwstate [in] the current state.
	 * @param pmap [in] properties to use - see #addDefaultProperties for details.
	 * @param log [in] (optional) add logging info.
	 * @param task [in] (optional) task to add work to.
	 */
	virtual void doCollisions(
			const RWPEBodyDynamic* rigidBody,
			const std::vector<const RWPEBodyFixed*> fixedBodies,
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
	 * This implementation uses RWPECollisionSolverSimultaneous to solve for pairs of bodies.
	 * See documentation for this class for default values and description of properties.
	 *
	 *  Property Name                            | Type   | Default value | Description
	 *  ---------------------------------------- | ------ | ------------- | -----------
	 *  -                                        | -      | -             | RWPECollisionSolverSimultaneous::addDefaultProperties defines properties used by this solver.
	 *
	 * @param map [in/out] the map to add properties to.
	 */
	virtual void addDefaultProperties(rw::common::PropertyMap& map) const;

private:
	class SolvePairTask;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPECOLLISIONSOLVERSINGLE_HPP_ */
