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

#ifndef RWSIM_CONTACTS_CONTACTDETECTORDATA_HPP_
#define RWSIM_CONTACTS_CONTACTDETECTORDATA_HPP_

/**
 * @file ContactDetectorData.hpp
 *
 * \copydoc rwsim::contacts::ContactDetectorData
 */

#include "ContactStrategyData.hpp"

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Container for data that is stored by a contact detector between contact detection calls.
 *
 * This will under normal circumstances be a collection of ContactStrategyData objects for the individual strategies used by the detector.
 *
 * Keeping this data between consecutive calls to the contact detector will allow strategies to exploit spatial and temporal coherence to
 * speed up algorithms.
 */
class ContactDetectorData {
public:
	/**
	 * @brief Get the strategy data for the strategy with given priority.
	 *
	 * @param priority [in] the priority to find strategy data for.
	 * @return the strategy data.
	 */
	ContactStrategyData getStrategyData(std::size_t priority) const;

	/**
	 * @brief Set the strategy data for the strategy with the given priority.
	 *
	 * @param priority [in] the priority to find strategy data for.
	 * @param data [in] the strategy data to set.
	 */
	void setStrategyData(std::size_t priority, const ContactStrategyData &data);

	/**
	 * @brief Add strategy data for the strategy with the given priority.
	 *
	 * @param priority [in] the priority to add strategy data for.
	 * @param data [in] the strategy data to set.
	 */
	void addStrategyData(std::size_t priority, const ContactStrategyData &data);

	/**
	 * @brief Remove strategy data for the strategy with the given priority.
	 *
	 * @param priority [in] the priority to remove strategy data for.
	 */
	void removeStrategyData(std::size_t priority);

	/**
	 * @brief Remove all data.
	 */
	void clear();

private:
	std::vector<ContactStrategyData> _stratData;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTDETECTORDATA_HPP_ */
