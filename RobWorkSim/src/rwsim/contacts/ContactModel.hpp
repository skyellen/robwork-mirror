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

#ifndef RWSIM_CONTACTS_CONTACTMODEL_HPP_
#define RWSIM_CONTACTS_CONTACTMODEL_HPP_

/**
 * @file rwsim/contacts/ContactModel.hpp
 *
 * \copydoc rwsim::contacts::ContactModel
 */

#include <rw/proximity/ProximityModel.hpp>

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief The ContactModel is an interface for the contact models implemented by different contact strategies.
 *
 * Each contact strategy requires different information for the geometries it uses.
 * This is a common interface for all of these types.
 */
class ContactModel: public rw::proximity::ProximityModel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactModel> Ptr;

	/**
	 * @brief Construct new contact model.
	 *
	 * @param pOwner [in] The contact strategy that creates this model.
	 */
	ContactModel(rw::proximity::ProximityStrategy* pOwner): ProximityModel(pOwner) {};

	/**
	 * @brief Destruct the model.
	 */
	virtual ~ContactModel() {};

	/**
	 * @brief Get name of model as a string.
	 *
	 * @return name of model.
	 */
	virtual std::string getName() const = 0;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTMODEL_HPP_ */
