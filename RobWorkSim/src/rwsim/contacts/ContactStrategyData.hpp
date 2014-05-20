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
 *
 * The ContactStrategy normally implements the SpecificData class if it wants to store data in
 * ContactStrategyData. The concrete implementation will depend on the strategy, and should under normal
 * circumstances not be directly accessible by the user.
 *
 * At a later stage ContactStrategyData might be extended to allow storing timing values for detailed benchmarks
 * of contact detection algorithms.
 */
class ContactStrategyData {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ContactStrategyData> Ptr;

	//! @brief Constructor.
	ContactStrategyData();

	//! @brief Copy data to new container.
	ContactStrategyData(const ContactStrategyData& data);

	//! @brief Destructor.
	virtual ~ContactStrategyData();

	/**
	 * @brief Assign data from other container to this container.
	 * @param data [in] the data to copy.
	 * @return reference to this container.
	 */
	ContactStrategyData& operator=(const ContactStrategyData& data);

	//! @brief Base class that can be extended to implement strategy-specific data.
	class SpecificData {
	public:
		//! @brief Constructor.
		SpecificData() {}

		//! @brief Destructor.
		virtual ~SpecificData() {}

		/**
		 * @brief Do a copy of the data.
		 * @return a new copy of the data owned by the caller.
		 */
		virtual SpecificData* copy() const = 0;
	};

	/**
	 * @brief Get the current strategy-specific data.
	 * @return a pointer to the data or NULL if none has been set.
	 */
	virtual SpecificData* getSpecificData() const;

	/**
	 * @brief Set the strategy-specific data.
	 * @param data [in] a pointer to the data to store.
	 */
	virtual void setSpecificData(SpecificData* data);

	/**
	 * @brief Check whether data container has been initialized with specific data.
	 * @return true if initialized.
	 */
	bool isInitialized() const;

private:
	SpecificData* _data;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYDATA_HPP_ */
