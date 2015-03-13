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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTISLANDSTATE_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTISLANDSTATE_HPP_

/**
 * @file TNTIslandState.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTIslandState
 */

#include "TNTBody.hpp"

#include <rw/math/Wrench6D.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>

#include <string>
#include <vector>

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTConstraint;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Stores the complete internal state of a TNTIsland.
 *
 * The TNTIslandState stores all internal information required to do a complete rollback for a TNTIsland.
 * The TNTIsland engine can also be reset manually to a given TNTIslandState.
 */
class TNTIslandState {
public:
	//! @brief Default constructor.
	TNTIslandState();

	//! @brief Copy constructor.
	TNTIslandState(const TNTIslandState& state);

	//! @brief Destructor.
	virtual ~TNTIslandState();

	/**
	 * @brief Assignment operator.
	 * @param state [in] the state to copy.
	 * @return reference to the updated object.
	 */
	TNTIslandState& operator=(const TNTIslandState &state);

	/**
	 * @brief Get the simulation time of this state.
	 * @return time.
	 */
	double getTime() const;

	/**
	 * @brief Change the time of this state.
	 * @param time [in] new time.
	 */
	void setTime(double time);

	/**
	 * @brief Get the length of the last timestep.
	 * @return timestep.
	 */
	double getLastTimeStep() const;

	/**
	 * @brief Change the length of the last timestep.
	 * @param timestep [in] new timestep.
	 */
	void setLastTimeStep(double timestep);

	/**
	 * @brief Get the number of repetitions.
	 * @return repetitions.
	 */
	std::size_t getRepetitions() const;

	/**
	 * @brief Change the number of repetitions already done.
	 * @param repetitions [in] new number of repetitions.
	 */
	void setRepetitions(std::size_t repetitions);


	/**
	 * @name Bodies.
	 * Functions related to bodies.
	 */
	///@{
	/**
	 * @brief Get the configuration for a specific TNTBody.
	 * @param body [in] the body to find configuration for.
	 * @return a pointer to the configuration.
	 */
	virtual TNTBody::Configuration* getConfiguration(const TNTBody* body) const;

	/**
	 * @brief Set a new configuration for a TNTBody.
	 * @note If a configuration was set previously, this configuration is deleted!
	 * @param body [in] the body to set configuration for.
	 * @param configuration [in] the configuration to set.
	 */
	virtual void setConfiguration(const TNTBody* body, TNTBody::Configuration* configuration);
	///@}

	/**
	 * @name Constraint wrenches.
	 * Functions related to constraints.
	 */
	///@{
	/**
	 * @brief Get the total wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrench(const TNTConstraint* constraint) const;

	/**
	 * @brief Get the total applied wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchApplied(const TNTConstraint* constraint) const;

	/**
	 * @brief Get the constraint wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchConstraint(const TNTConstraint* constraint) const;

	/**
	 * @brief Change the applied wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @param wrench [in] new wrench in world coordinates.
	 */
	virtual void setWrenchApplied(const TNTConstraint* constraint, const rw::math::Wrench6D<> wrench);

	/**
	 * @brief Change the constraint wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @param wrench [in] new wrench in world coordinates.
	 */
	virtual void setWrenchConstraint(const TNTConstraint* constraint, const rw::math::Wrench6D<> wrench);
	///@}

	/**
	 * @name Contacts.
	 * Functions related to contacts and tracking of contacts.
	 */
	///@{
	/**
	 * @brief Get contact data.
	 * @return contact data.
	 */
	virtual rwsim::contacts::ContactDetectorData getContactData() const;

	/**
	 * @brief Set contact data.
	 * @param data [in] the contact data to set.
	 */
	virtual void setContactData(const rwsim::contacts::ContactDetectorData& data);

	/**
	 * @brief Get a list of all contacts.
	 * @return list of new contacts.
	 */
	virtual std::vector<rwsim::contacts::Contact> getContacts() const;

	/**
	 * @brief Get tracking data for all contacts.
	 * @return tracking data.
	 */
	virtual rwsim::contacts::ContactDetectorTracking getContactsTracking() const;

	/**
	 * @brief Check if there are any contacts stored in state.
	 * @return true if there are contacts, false if not.
	 */
	virtual bool hasContacts() const;

	/**
	 * @brief Set the list of contacts.
	 * @param contacts [in] list of contacts.
	 * @param data [in] the tracking data.
	 */
	virtual void setContacts(const std::vector<rwsim::contacts::Contact>& contacts, const rwsim::contacts::ContactDetectorTracking& data);
	///@}

	/**
	 * @name Temporary Constraints.
	 * Functions related to the temporary constraints (the contact constraints).
	 */
	///@{
	/**
	 * @brief Get list of temporary constraints.
	 * @return list of constraints.
	 */
	std::list<TNTConstraint*> getTemporaryConstraints() const;

	/**
	 * @brief Get a list of temporary constraints on specific body.
	 * @param body [in] the body.
	 * @return list of constraints for body.
	 */
	std::list<const TNTConstraint*> getTemporaryConstraints(const TNTBody* body) const;

	/**
	 * @brief Check if any contacts are currently registered in the state.
	 * @return true if there are contact constraints, false if not.
	 */
	bool hasTemporaryConstraints() const;

	/**
	 * @brief Add a new temporary constraint.
	 * @param constraint [in] the constraint.
	 */
	void addTemporaryConstraint(TNTConstraint* constraint);

	/**
	 * @brief Remove a temporary constraint.
	 * @param constraint [in] the constraint to remove.
	 */
	void removeTemporaryConstraint(const TNTConstraint* constraint);

	//! @brief Clear all temporary constraints.
	void clearTemporaryConstraints();
	///@}

private:
	double _time;
	double _timestep;
	std::size_t _repeated;

	std::map<const rwsim::dynamics::Body*, const TNTBody*> _rwBodyToBody;
	std::map<const TNTBody*, TNTBody::Configuration*> _bodyToConfiguration;

	std::map<const TNTConstraint*, rw::math::Wrench6D<> > _constraintToAppliedWrench;
	std::map<const TNTConstraint*, rw::math::Wrench6D<> > _constraintToConstraintWrench;

	rwsim::contacts::ContactDetectorData _contactData;

	std::vector<rwsim::contacts::Contact> _contacts;
	rwsim::contacts::ContactDetectorTracking _tracking;

	std::list<TNTConstraint*> _tempConstraints;
	std::map<const TNTBody*, std::list<const TNTConstraint*> > _bodyToTempConstraints;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTISLANDSTATE_HPP_ */
