/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYDATA_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYDATA_HPP_

/**
 * @file ContactStrategyData.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyData
 */

#include "Contact.hpp"

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Container for data that is stored by contact strategies between contact detection calls.
 *
 * This will in general be a list of the previous contacts detected by the strategy. Special implementations can however be used by
 * strategies that need to store more data.
 *
 * Keeping this data between consecutive calls to the contact strategy will allow strategies to exploit spatial and temporal coherence to
 * speed up algorithms.
 */
class ContactStrategyData {
public:
	/**
	 * @brief Get list of contacts.
	 *
	 * @return list of contacts.
	 */
	std::vector<Contact> getContacts();

	/**
	 * @brief Set list of contacts.
	 *
	 * @param contacts [in] list of contacts.
	 */
	void setContacts(const std::vector<Contact> &contacts);

private:
	std::vector<Contact> _contacts;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYDATA_HPP_ */
