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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTBODYCONTACTMANAGER_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTBODYCONTACTMANAGER_HPP_

/**
 * @file TNTBodyContactManager.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTBodyContactManager
 */

#include <list>
#include <vector>
#include <utility>
#include <map>

// Forward declarations
namespace rw { namespace common { template <class T> class Ptr; }};
namespace rw { namespace kinematics { class Frame; }};
namespace rwsim { namespace dynamics { class DynamicWorkCell; }};

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBody;
class TNTKinematicBody;
class TNTRigidBody;
class TNTConstraint;
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Manager that maintains lists of all bodies and constraints in the system, as well
 * as maps of how the bodies are connected by constraints.
 *
 * The purpose of the manager is to make it simple to add and remove bodies dynamically, and
 * provides functions for easily obtaining relevant subsets of the constraints or bodies in the
 * system based on how the bodies are connected by constraints.
 */
class TNTBodyConstraintManager {
public:
	//! @brief Type of the body lists.
	typedef std::list<const TNTBody*> BodyList;
	//! @brief Type of the dynamic bodies lists.
	typedef std::list<const TNTRigidBody*> DynamicBodyList;
	//! @brief Type of the kinematic bodies lists.
	typedef std::list<const TNTKinematicBody*> KinematicBodyList;
	//! @brief Type of the constraint lists with non-constant objects.
	typedef std::list<TNTConstraint*> ConstraintList;
	//! @brief Type of the constraint lists with constant objects.
	typedef std::list<const TNTConstraint*> ConstraintListConst;
	//! @brief Type of the body lists.
	typedef std::pair<std::vector<const TNTBody*>, std::vector<const TNTConstraint*> > Component;

	//! @brief Construct empty manager.
	TNTBodyConstraintManager();

	//! @brief Destructor.
	virtual ~TNTBodyConstraintManager();

	/**
	 * @brief Initialize the manager from a dynamic workcell.
	 *
	 * All objects and constraints defined in the dynamic workcell will be added.
	 *
	 * @param dwc [in] the DynamicWorkCell
	 */
	void initFromDWC(const rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell>& dwc);

	/**
	 * @brief Get a list of all bodies.
	 * @return list of bodies.
	 */
	BodyList getBodies() const;

	/**
	 * @brief Get a list of the rigid bodies only.
	 * @return list of rigid bodies.
	 */
	DynamicBodyList getDynamicBodies() const;

	/**
	 * @brief Get a list of the kinematic bodies only.
	 * @return list of kinematic bodies.
	 */
	KinematicBodyList getKinematicBodies() const;

	/**
	 * @brief Get a list of persistent constraints.
	 *
	 * Persistent constraints are constraints that does not change during a simulation step.
	 * Hence TNTContact constraints will not be a part of this list.
	 *
	 * @return a list of constraints.
	 */
	ConstraintList getPersistentConstraints() const;

	/**
	 * @brief Get a list of constraints with the temporary constraints.
	 * @param state [in] the TNTIslandState to get the temporary constraints from.
	 * @return list of all constraints.
	 */
	ConstraintList getConstraints(const TNTIslandState& state) const;

	/**
	 * @brief Get all constraints for a given body.
	 * @param body [in] the body to find constraints for.
	 * @param state [in] the TNTIslandState where temporary constraints are stored.
	 * @return a list of constraints (constant objects).
	 */
	ConstraintListConst getConstraints(const TNTBody* body, const TNTIslandState& state) const;

	/**
	 * @brief Get all constraints for a given pair of bodies.
	 * @param bodyA [in] the first body to find constraints for.
	 * @param bodyB [in] the second body to find constraints for.
	 * @param state [in] the TNTIslandState where temporary constraints are stored.
	 * @return a list of constraints (constant objects).
	 */
	ConstraintListConst getConstraints(const TNTBody* bodyA, const TNTBody* bodyB, const TNTIslandState& state) const;

	/**
	 * @brief Get the temporary constraints.
	 * @param state [in] state where constraints are stored.
	 * @return list of constraints.
	 */
	ConstraintList getTemporaryConstraints(const TNTIslandState* state) const;

	/**
	 * @brief Add a new body to the manager.
	 * @param body [in] the body to add.
	 */
	void addBody(const TNTBody* const body);

	/**
	 * @brief Add a new persistent constraint to the manager.
	 * @param constraint [in] the constraint to add.
	 */
	void addConstraint(TNTConstraint* constraint);

	/**
	 * @brief Add temporary constraint.
	 * @param constraint [in] the constraint to add.
	 * @param state [in/out] the state to store the temporary constraint in.
	 */
	void addTemporaryConstraint(TNTConstraint* constraint, TNTIslandState& state) const;

	/**
	 * @brief Remove a body.
	 * @param body [in] the body to remove.
	 */
	void removeBody(const TNTBody* const body);

	/**
	 * @brief Remove persistens constraint.
	 * @param constraint [in] the constraint to remove.
	 */
	void removeConstraint(TNTConstraint* constraint);

	/**
	 * @brief Remove a temporary constraint.
	 * @param constraint [in] the constraint to remove.
	 * @param state [in/out] the state to update.
	 */
	void removeTemporaryConstraint(const TNTConstraint* constraint, TNTIslandState& state) const;

	/**
	 * @brief Clear all temporary constraints.
	 * @param state [in] the state where temporary constraints are saved.
	 */
	void clearTemporaryConstraints(TNTIslandState& state) const;

	/**
	 * @brief Get the body for a given frame.
	 * @param frame [in] the frame to find body for.
	 * @return the body or NULL if not found.
	 */
	const TNTBody* getBody(const rw::kinematics::Frame* frame) const;

private:
	BodyList _allBodies;
	DynamicBodyList _dynamicBodies;
	KinematicBodyList _kinematicBodies;
	ConstraintList _constraints;
	std::map<const TNTBody*, ConstraintListConst> _bodyToConstraints;
	std::map<const rw::kinematics::Frame*, const TNTBody*> _frameToBody;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTBODYCONTACTMANAGER_HPP_ */
