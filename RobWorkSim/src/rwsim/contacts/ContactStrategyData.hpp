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
 * Keeping data between consecutive calls to the contact strategy will allow strategies to exploit spatial and
 * temporal coherence to speed up algorithms.
 */
class ContactStrategyData {
public:
	//! @brief Constructor.
	ContactStrategyData();

	//! @brief Destructor.
	virtual ~ContactStrategyData();

	/**
	 * @brief Do a copy of the data.
	 * @return a new copy of the data owned by the caller.
	 */
	virtual ContactStrategyData* copy() const;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYDATA_HPP_ */
