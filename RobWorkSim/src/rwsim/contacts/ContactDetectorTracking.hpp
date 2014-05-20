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

#ifndef RWSIM_CONTACTS_CONTACTDETECTORTRACKING_HPP_
#define RWSIM_CONTACTS_CONTACTDETECTORTRACKING_HPP_

/**
 * @file ContactDetectorTracking.hpp
 *
 * \copydoc rwsim::contacts::ContactDetectorTracking
 */

#include "ContactStrategyTracking.hpp"

#include <rw/common/Ptr.hpp>
#include <vector>
#include <map>

// Forward declarations
namespace rw { namespace kinematics { class Frame; } }

namespace rwsim {
namespace contacts {

// Forward declarations
class ContactModel;
class ContactStrategy;

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Container for meta-data that can be used to track contact across multiple calls to contact detector,
 * and allows attaching user specified data to the contact.
 */
class ContactDetectorTracking {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ContactDetectorTracking> Ptr;

	//! @brief Constructor.
	ContactDetectorTracking();

	//! @brief Copy data to new container.
	ContactDetectorTracking(const ContactDetectorTracking& data);

	//! @brief Destructor.
	virtual ~ContactDetectorTracking();

	/**
	 * @brief Assign data from other container to this container.
	 * @param data [in] the data to copy.
	 * @return reference to this container.
	 */
	ContactDetectorTracking& operator=(const ContactDetectorTracking& data);

	//! @brief Clear all stored meta-data.
	void clear();

	/**
	 * @brief Remove meta-data for a specific contact.
	 * @param index [in] the contact to remove.
	 */
	void remove(std::size_t index);

	/**
	 * @brief Get user data for the contact with given index.
	 * @param index [in] the contact to get user data for.
	 * @return Pointer to user data, or NULL if no user data is set.
	 */
	ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const;

	/**
	 * @brief Get user data for all contacts.
	 * @return list of pointers to user data - can be NULL if none is set.
	 */
	std::vector<ContactStrategyTracking::UserData::Ptr> getUserData() const;

	/**
	 * @brief Attach user data to a given contact.
	 * @param index [in] the contact to set user data for.
	 * @param data [in] a pointer to the data.
	 */
	void setUserData(std::size_t index, ContactStrategyTracking::UserData::Ptr data);

	/**
	 * @brief Set user data for all contacts.
	 * @param data [in] list of pointers to data.
	 */
	void setUserData(const std::vector<ContactStrategyTracking::UserData::Ptr> &data);

	/**
	 * @brief Get the number of contacts tracked.
	 * @return the number of contacts.
	 */
	std::size_t getSize() const;

	//! @brief Meta-data for a contact that allows it to be recalculated.
	struct ContactInfo {
		//! @brief Constructor.
		ContactInfo(): tracking(NULL), id(0), total(0) {}
		//! @brief The frame pair the contact can be calculated from.
		std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*> frames;
		//! @brief The contact models the contact can be calculated from.
		std::pair<ContactModel*,ContactModel*> models;
		//! @brief The strategy that found the contact.
		rw::common::Ptr<ContactStrategy> strategy;
		//! @brief Tracking data for this specific pair of contact models (multiple contacts can point to the same).
		ContactStrategyTracking* tracking;
		//! @brief A counter if multiple contacts uses the same ContactStrategyTracking (these will then be in order).
		std::size_t id;
		//! @brief The total number of contacts that where found using the same ContactStrategyTracking.
		std::size_t total;
	};

	/**
	 * @name Functions for internal usage.
	 * These functions should under normal circumstances not be used by the user of a ContactDetector.
	 */
	///@{
	/**
	 * @brief Get a reference to the stored meta-data.
	 * @return reference to a vector of ContactInfo.
	 */
	std::vector<ContactInfo>& getInfo();

	/**
	 * @brief Get a reference to the stored meta-data.
	 * @return reference to a vector of constant ContactInfo.
	 */
	const std::vector<ContactInfo>& getInfo() const;

	/**
	 * @brief Get the stored ContactStrategyTracking for a specific pair of ContactModels.
	 * @param modelA [in] the first ContactModel.
	 * @param modelB [in] the second ContactModel.
	 * @return a pointer to the ContactStrategyTracking - caller does NOT own the pointer.
	 */
	ContactStrategyTracking& getStrategyTracking(const ContactModel* modelA, const ContactModel* modelB);
	///@}

private:
	std::vector<ContactInfo> _info;
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyTracking> > _modelPairToTracking;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTDETECTORTRACKING_HPP_ */
