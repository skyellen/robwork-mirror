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

#include <map>

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
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ContactDetectorData> Ptr;

	//! @brief Constructor.
	ContactDetectorData();

	//! @brief Copy data to new container.
	ContactDetectorData(const ContactDetectorData& data);

	//! @brief Destructor.
	virtual ~ContactDetectorData();

	/**
	 * @brief Assign data from other container to this container.
	 * @param data [in] the data to copy.
	 * @return reference to this container.
	 */
	ContactDetectorData& operator=(const ContactDetectorData& data);

	//! @brief Remove all data.
	void clear();

	/**
	 * @brief Get the stored ContactStrategyData for a specific pair of ContactModels.
	 * @param modelA [in] the first ContactModel.
	 * @param modelB [in] the second ContactModel.
	 * @return a reference to the ContactStrategyData.
	 */
	ContactStrategyData& getStrategyData(const ContactModel* modelA, const ContactModel* modelB);

private:
	std::map<const ContactModel*, std::map<const ContactModel*, ContactStrategyData> > _modelPairToData;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTDETECTORDATA_HPP_ */
