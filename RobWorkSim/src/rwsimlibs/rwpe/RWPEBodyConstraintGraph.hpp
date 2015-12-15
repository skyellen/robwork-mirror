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

#ifndef RWSIMLIBS_RWPE_RWPEBODYCONTACTMANAGER_HPP_
#define RWSIMLIBS_RWPE_RWPEBODYCONTACTMANAGER_HPP_

/**
 * @file RWPEBodyConstraintGraph.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEBodyConstraintGraph
 */

#include <list>
#include <vector>
#include <utility>
#include <map>
#include <set>
#include <string>

// Forward declarations
namespace rw { namespace common { template <class T> class Ptr; }};
namespace rw { namespace kinematics { class Frame; }};
namespace rwsim { namespace dynamics { class DynamicWorkCell; }};

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEBody;
class RWPEBodyKinematic;
class RWPEBodyDynamic;
class RWPEConstraint;
class RWPEIslandState;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Manager that maintains lists of all bodies and constraints in the system, as well
 * as maps of how the bodies are connected by constraints.
 *
 * The purpose of the manager is to make it simple to add and remove bodies dynamically, and
 * provides functions for easily obtaining relevant subsets of the constraints or bodies in the
 * system based on how the bodies are connected by constraints.
 */
class RWPEBodyConstraintGraph {
public:
	//! @brief Type of the body lists.
	typedef std::list<const RWPEBody*> BodyList;
	//! @brief Type of the dynamic bodies lists.
	typedef std::list<const RWPEBodyDynamic*> DynamicBodyList;
	//! @brief Type of the kinematic bodies lists.
	typedef std::list<const RWPEBodyKinematic*> KinematicBodyList;
	//! @brief Type of the constraint lists with non-constant objects.
	typedef std::list<RWPEConstraint*> ConstraintList;
	//! @brief Type of the constraint lists with constant objects.
	typedef std::list<const RWPEConstraint*> ConstraintListConst;

	//! @brief Construct empty manager.
	RWPEBodyConstraintGraph();

	//! @brief Destructor.
	virtual ~RWPEBodyConstraintGraph();

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
	 * Hence RWPEContact constraints will not be a part of this list.
	 *
	 * @return a list of constraints.
	 */
	ConstraintList getPersistentConstraints() const;

	/**
	 * @brief Get a list of constraints with the temporary constraints.
	 * @param state [in] the RWPEIslandState to get the temporary constraints from.
	 * @return list of all constraints.
	 */
	ConstraintList getConstraints(const RWPEIslandState& state) const;

	/**
	 * @brief Get all constraints for a given body.
	 * @param body [in] the body to find constraints for.
	 * @param state [in] the RWPEIslandState where temporary constraints are stored.
	 * @return a list of constraints (constant objects).
	 */
	ConstraintListConst getConstraints(const RWPEBody* body, const RWPEIslandState& state) const;

	/**
	 * @brief Get all constraints for a given pair of bodies.
	 * @param bodyA [in] the first body to find constraints for.
	 * @param bodyB [in] the second body to find constraints for.
	 * @return a list of constraints.
	 */
	ConstraintList getConstraints(const RWPEBody* bodyA, const RWPEBody* bodyB) const;

	/**
	 * @brief Get all constraints for a given pair of bodies.
	 * @param bodyA [in] the first body to find constraints for.
	 * @param bodyB [in] the second body to find constraints for.
	 * @param state [in] the RWPEIslandState where temporary constraints are stored.
	 * @return a list of constraints (constant objects).
	 */
	ConstraintListConst getConstraints(const RWPEBody* bodyA, const RWPEBody* bodyB, const RWPEIslandState& state) const;

	/**
	 * @brief Get the temporary constraints.
	 * @param state [in] state where constraints are stored.
	 * @return list of constraints.
	 */
	ConstraintList getTemporaryConstraints(const RWPEIslandState* state) const;

	/**
	 * @brief Check if there are any contacts or constraints.
	 * @param state [in] state where contacts are stored.
	 * @return true if there are any contacts or constraints.
	 */
	bool hasContactsOrConstraints(const RWPEIslandState& state) const;

	/**
	 * @brief Add a new body to the manager.
	 * @param body [in] the body to add.
	 */
	void addBody(const RWPEBody* const body);

	/**
	 * @brief Add a set of new body to the manager.
	 * @param bodies [in] the bodies to add.
	 */
	void addBodies(const BodyList& bodies);

	/**
	 * @brief Add a new persistent constraint to the manager.
	 * @param constraint [in] the constraint to add.
	 */
	void addConstraint(RWPEConstraint* constraint);

	/**
	 * @brief Add new persistent constraints to the manager.
	 * @param constraints [in] the constraints to add.
	 */
	void addConstraints(const ConstraintList& constraints);

	/**
	 * @brief Add temporary constraint.
	 * @param constraint [in] the constraint to add.
	 * @param state [in/out] the state to store the temporary constraint in.
	 */
	void addTemporaryConstraint(RWPEConstraint* constraint, RWPEIslandState& state) const;

	/**
	 * @brief Remove a body.
	 * @param body [in] the body to remove.
	 */
	void removeBody(const RWPEBody* const body);

	/**
	 * @brief Remove persistens constraint.
	 * @param constraint [in] the constraint to remove.
	 */
	void removeConstraint(RWPEConstraint* constraint);

	/**
	 * @brief Remove a temporary constraint.
	 * @param constraint [in] the constraint to remove.
	 * @param state [in/out] the state to update.
	 */
	void removeTemporaryConstraint(const RWPEConstraint* constraint, RWPEIslandState& state) const;

	/**
	 * @brief Clear all temporary constraints.
	 * @param state [in] the state where temporary constraints are saved.
	 */
	void clearTemporaryConstraints(RWPEIslandState& state) const;

	/**
	 * @brief Get the body for a given frame.
	 * @param frame [in] the frame to find body for.
	 * @return the body or NULL if not found.
	 */
	const RWPEBody* getBody(const rw::kinematics::Frame* frame) const;

	/**
	 * @brief Get the body with given name.
	 * @param name [in] the name of the body.
	 * @return the body or NULL if not found.
	 */
	const RWPEBody* getBody(const std::string& name) const;

	/**
	 * @brief Check if body is added.
	 * @param body [in] the body to check for.
	 * @return true if body was found, false otherwise.
	 */
	bool has(const RWPEBody* body) const;

	/**
	 * @brief Get the dynamic components using a list of bodies in contact.
	 *
	 * A dynamic component is composed of dynamic bodies directly connected to each other
	 * by contacts or constraints. Fixed or kinematic bodies are included in
	 * the component if they are connected to the dynamic bodies in the component.
	 *
	 * This means that static bodies have the effect of increasing the number of
	 * dynamic components, making each component smaller. This is a benefit as it makes the
	 * underlying dynamics problem easier to solve and parallelize.
	 *
	 * Each dynamic body is guaranteed to only lie in one component, but the static bodies can
	 * be included in multiple components simultaneously.
	 *
	 * Note the this function is different than the dynamic components calculated using
	 * a RWPEIslandState. When this state information is used, full 6D springs will be able
	 * to decouple two components. As springs can impose constraints dynamically, they must
	 * in this function be considered as ordinary constraints.
	 *
	 * @param pairs [in] the list of body pairs.
	 * @return a set of dynamic components owned by the caller.
	 */
	std::set<RWPEBodyConstraintGraph*> getDynamicComponents(const std::list<std::pair<const RWPEBody*, const RWPEBody*> >& pairs) const;

	/**
	 * @brief Get the dynamic components using dynamic contact info.
	 *
	 * A dynamic component is composed of dynamic bodies directly connected to each other
	 * by contacts or constraints. Constraints must however be constrained in velocity space
	 * in at least one dimension to be considered. Fixed or kinematic bodies are included in
	 * the component if they are connected to the dynamic bodies in the component.
	 *
	 * This means that springs and static bodies have the effect of increasing the number of
	 * dynamic components, making each component smaller. This is a benefit as it makes the
	 * underlying dynamics problem easier to solve and parallelize.
	 *
	 * Each dynamic body is guaranteed to only lie in one component, but the static bodies can
	 * be included in multiple components simultaneously. Be aware that free constraints (that
	 * are not constrained in velocity space) is not included in any component.
	 *
	 * @param state [in] the state where contact constraints are saved.
	 * @return a set of dynamic components owned by the caller.
	 */
	std::set<RWPEBodyConstraintGraph*> getDynamicComponents(const RWPEIslandState& state) const;

	/**
	 * @brief Get components that are connected by the given constraints.
	 *
	 * Connected components tries to connect the given by constraints.
	 * Constraints sharing the same end-bodies will be connected in the same component.
	 *
	 * Note that the components found by this function might include more constraints than
	 * the number of constraints given as input. This is due to the fact that a pair of objects
	 * can have more than one contact and/or constraint. Even though the input only mentions one
	 * of these, all other contacts and constraints will also be included in the component.
	 *
	 * @param constraints [in] list of constraints to search from.
	 * @return a set of connected components owned by the caller.
	 */
	std::set<RWPEBodyConstraintGraph*> getConnectedComponents(const ConstraintListConst& constraints) const;

private:
	RWPEBodyConstraintGraph(const RWPEBodyConstraintGraph* parent);

	BodyList _allBodies;
	DynamicBodyList _dynamicBodies;
	KinematicBodyList _kinematicBodies;
	ConstraintList _constraints;
	std::map<const RWPEBody*, ConstraintListConst> _bodyToConstraints;
	std::map<const rw::kinematics::Frame*, const RWPEBody*> _frameToBody;
	const RWPEBodyConstraintGraph* const _parent;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEBODYCONTACTMANAGER_HPP_ */
