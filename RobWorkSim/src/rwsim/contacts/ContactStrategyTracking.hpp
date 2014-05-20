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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYTRACKING_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYTRACKING_HPP_

/**
 * @file ContactStrategyTracking.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyTracking
 */

#include <rw/common/Ptr.hpp>

namespace rwsim {
namespace contacts {

// Forward declarations
class ContactStrategyTracking;

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Interface for a container of meta-data that can be used to track contact across multiple
 * calls to a contact strategy, and allows attaching user specified data to the contact.
 *
 * Each ContactStrategy implementation should also implement the ContactStrategyTracking interface,
 * to allow tracking contacts.
 */
class ContactStrategyTracking {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ContactStrategyTracking> Ptr;

	//! @brief Constructor.
	ContactStrategyTracking();

	/**
	 * @brief Copy constructor.
	 * @param tracking [in] the tracking information to copy.
	 */
	ContactStrategyTracking(const ContactStrategyTracking& tracking);

	//! @brief Destructor.
	virtual ~ContactStrategyTracking();

	/**
	 * @brief Assign data from other container to this container.
	 * @param data [in] the data to copy.
	 * @return reference to this container.
	 */
	ContactStrategyTracking& operator=(const ContactStrategyTracking& data);

	//! @brief Base struct that can be extended for user specific data.
	struct UserData {
		//! @brief Smart pointer type.
		typedef rw::common::Ptr<const UserData> Ptr;

		//! @brief Constructor.
		UserData() {}

		//! @brief Destructor.
		virtual ~UserData() {}
	};

	/**
	 * @brief Get user data for the contact with given index.
	 * @param index [in] the contact to get user data for.
	 * @return pointer to user data, or NULL if no user data is set.
	 */
	virtual const UserData::Ptr getUserData(std::size_t index) const;

	/**
	 * @brief Attach user data to a given contact.
	 * @param index [in] the contact to set user data for.
	 * @param data [in] a pointer to the data.
	 */
	virtual void setUserData(std::size_t index, const UserData::Ptr data);

	/**
	 * @brief Remove meta-data for a specific contact.
	 * @param index [in] the contact to remove.
	 */
	virtual void remove(std::size_t index);

	//! @brief Clear all tracking information.
	virtual void clear();

	/**
	 * @brief Get the number of contacts tracked currently.
	 * @return the number of tracked contacts.
	 */
	virtual std::size_t getSize() const;

	//! @brief Base struct that can be extended for strategy specific data.
	struct StrategyData {
		//! @brief Constructor.
		StrategyData() {}

		//! @brief Destructor.
		virtual ~StrategyData() {}

		/**
		 * @brief Do a copy of the strategy data.
		 * @return a new copy of the strategy data owned by the caller.
		 */
		virtual StrategyData* copy() const = 0;

		//! @copydoc rwsim::contacts::ContactStrategyTracking::getUserData
		virtual const UserData::Ptr getUserData(std::size_t index) const = 0;

		//! @copydoc rwsim::contacts::ContactStrategyTracking::setUserData
		virtual void setUserData(std::size_t index, const UserData::Ptr data) = 0;

		//! @copydoc rwsim::contacts::ContactStrategyTracking::remove
		virtual void remove(std::size_t index) = 0;

		//! @copydoc rwsim::contacts::ContactStrategyTracking::getSize
		virtual std::size_t getSize() const = 0;
	};

	/**
	 * @brief Get the current strategy data.
	 * @return pointer to StrategyData, or NULL if none set.
	 */
	virtual StrategyData* getStrategyData() const;

	/**
	 * @brief Set the strategy data to store for tracking purposes.
	 * @param data [in] a pointer to the strategy data.
	 */
	virtual void setStrategyData(StrategyData* data);

	/**
	 * @brief Check whether data container has been initialized with strategy data.
	 * @return true if initialized - NULL if not initialized, or if tracking is unsupported by strategy.
	 */
	virtual bool isInitialized() const;

private:
	StrategyData* _strategyData;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYTRACKING_HPP_ */
