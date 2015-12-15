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

#ifndef RWSIMLIBS_RWPE_RWPEISLANDSTATE_HPP_
#define RWSIMLIBS_RWPE_RWPEISLANDSTATE_HPP_

/**
 * @file RWPEIslandState.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEIslandState
 */

#include "RWPEBody.hpp"

#include <rw/math/Wrench6D.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>

#include <string>
#include <vector>
#include <list>

namespace rwsimlibs {
namespace rwpe {

// Forward declarations
class RWPEConstraint;
class RWPEContact;
class RWPEFrictionModelData;

//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Stores the complete internal state of a RWPEIsland.
 *
 * The RWPEIslandState stores all internal information required to do a complete rollback for a RWPEIsland.
 * The RWPEIsland engine can also be reset manually to a given RWPEIslandState.
 */
class RWPEIslandState {
public:
	//! @brief Default constructor.
	RWPEIslandState();

	//! @brief Copy constructor.
	RWPEIslandState(const RWPEIslandState& state);

	//! @brief Destructor.
	virtual ~RWPEIslandState();

	/**
	 * @brief Assignment operator.
	 * @param state [in] the state to copy.
	 * @return reference to the updated object.
	 */
	RWPEIslandState& operator=(const RWPEIslandState &state);

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
	 * @brief Get the configuration for a specific RWPEBody.
	 * @param body [in] the body to find configuration for.
	 * @return a pointer to the configuration.
	 */
	virtual RWPEBody::Configuration* getConfiguration(const RWPEBody* body) const;

	/**
	 * @brief Set a new configuration for a RWPEBody.
	 * @note If a configuration was set previously, this configuration is deleted!
	 * @param body [in] the body to set configuration for.
	 * @param configuration [in] the configuration to set.
	 */
	virtual void setConfiguration(const RWPEBody* body, RWPEBody::Configuration* configuration);
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
	virtual rw::math::Wrench6D<> getWrench(const RWPEConstraint* constraint) const;

	/**
	 * @brief Get the total applied wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchApplied(const RWPEConstraint* constraint) const;

	/**
	 * @brief Get the constraint wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @return the wrench in world coordinates.
	 */
	virtual rw::math::Wrench6D<> getWrenchConstraint(const RWPEConstraint* constraint) const;

	/**
	 * @brief Change the applied wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @param wrench [in] new wrench in world coordinates.
	 */
	virtual void setWrenchApplied(const RWPEConstraint* constraint, const rw::math::Wrench6D<> wrench);

	/**
	 * @brief Change the constraint wrench for a constraint.
	 * @param constraint [in] the constraint.
	 * @param wrench [in] new wrench in world coordinates.
	 */
	virtual void setWrenchConstraint(const RWPEConstraint* constraint, const rw::math::Wrench6D<> wrench);
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
	 * @name Frictional state for contacts.
	 * Functions related to the temporary constraints (the contact constraints).
	 */
	///@{
	/**
	 * @brief Get friction data associated to specific contact.
	 * @param contact [in] a pointer to the contact to find data for.
	 * @return a pointer (not owned by caller) to the friction data, or NULL if no data is registered.
	 */
	RWPEFrictionModelData* getFrictionData(const RWPEContact* contact) const;

	/**
	 * @brief Set new friction data for the contact.
	 * @param contact [in] a pointer to the contact to set data for.
	 * @param data [in] a pointer to the friction data (this function takes ownership).
	 */
	void setFrictionData(const RWPEContact* contact, RWPEFrictionModelData* data);

	/**
	 * @brief Destroy data associated to contact.
	 * @param contact [in] the contact to destroy friction data for.
	 */
	void clearFrictionData(const RWPEContact* contact);
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
	std::list<RWPEConstraint*> getTemporaryConstraints() const;

	/**
	 * @brief Get a list of temporary constraints on specific body.
	 * @param body [in] the body.
	 * @return list of constraints for body.
	 */
	std::list<const RWPEConstraint*> getTemporaryConstraints(const RWPEBody* body) const;

	/**
	 * @brief Check if any contacts are currently registered in the state.
	 * @return true if there are contact constraints, false if not.
	 */
	bool hasTemporaryConstraints() const;

	/**
	 * @brief Add a new temporary constraint.
	 * @param constraint [in] the constraint.
	 */
	void addTemporaryConstraint(RWPEConstraint* constraint);

	/**
	 * @brief Remove a temporary constraint.
	 * @param constraint [in] the constraint to remove.
	 */
	void removeTemporaryConstraint(const RWPEConstraint* constraint);

	//! @brief Clear all temporary constraints.
	void clearTemporaryConstraints();
	///@}

private:
	double _time;
	double _timestep;
	std::size_t _repeated;

	std::map<const rwsim::dynamics::Body*, const RWPEBody*> _rwBodyToBody;
	std::map<const RWPEBody*, RWPEBody::Configuration*> _bodyToConfiguration;

	std::map<const RWPEConstraint*, rw::math::Wrench6D<> > _constraintToAppliedWrench;
	std::map<const RWPEConstraint*, rw::math::Wrench6D<> > _constraintToConstraintWrench;

	rwsim::contacts::ContactDetectorData _contactData;

	std::vector<rwsim::contacts::Contact> _contacts;
	rwsim::contacts::ContactDetectorTracking _tracking;

	std::map<const RWPEContact*, RWPEFrictionModelData*> _contactToFrictionData;

	std::list<RWPEConstraint*> _tempConstraints;
	std::map<const RWPEBody*, std::list<const RWPEConstraint*> > _bodyToTempConstraints;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEISLANDSTATE_HPP_ */
