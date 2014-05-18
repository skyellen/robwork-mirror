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

#include <cstddef>

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
	//! @brief Constructor.
	ContactStrategyTracking() {};

	//! @brief Destructor.
	virtual ~ContactStrategyTracking() {};

	/**
	 * @brief Get user data for the contact with given index.
	 * @param index [in] the contact to get user data for.
	 * @return Pointer to user data, or NULL if no user data is set.
	 */
	virtual const void* getUserData(std::size_t index) const = 0;

	/**
	 * @brief Attach user data to a given contact.
	 * @param index [in] the contact to set user data for.
	 * @param data [in] a pointer to the data.
	 */
	virtual void setUserData(std::size_t index, const void* data) = 0;

	/**
	 * @brief Remove meta-data for a specific contact.
	 * @param index [in] the contact to remove.
	 */
	virtual void remove(std::size_t index) = 0;

	/**
	 * @brief Do a copy of the tracking data.
	 * @return a new copy of the tracking data owned by the caller.
	 */
	virtual ContactStrategyTracking* copy() const = 0;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYTRACKING_HPP_ */
