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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTCONTACT_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTCONTACT_HPP_

/**
 * @file TNTContact.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTContact
 */

#include "TNTConstraint.hpp"

#include <rw/math/Vector3D.hpp>
#include <rwsim/contacts/Contact.hpp>

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A special form of constraint that changes continuously during simulation.
 */
class TNTContact: public TNTConstraint {
public:
	//! @brief Contact constraint types
	typedef enum Type {
		Sticking = 1,//!< Sticking - the relative velocity is zero.
		Sliding = 2  //!< Sliding - force will be added opposite to the relative velocity.
	} Type;

	/**
	 * @brief Construct new unspecified contact between two bodies.
	 * @param parent [in] first body.
	 * @param child [in] second body.
	 */
	TNTContact(const TNTBody* parent, const TNTBody* child);

	/**
	 * @brief Construct new specified contact between two bodies.
	 * @param parent [in] first body.
	 * @param child [in] second body.
	 * @param contact [in] the contact.
	 * @param state [in] the state where the contact was found.
	 */
	TNTContact(const TNTBody* parent, const TNTBody* child, const rwsim::contacts::Contact &contact, const rw::kinematics::State &state);

	//! @brief Destructor.
	~TNTContact();

	/**
	 * @brief Check if contact is currently set to leaving.
	 * @return true if leaving, false otherwise.
	 */
	bool isLeaving() const;

	/**
	 * @brief Get the linear type of contact.
	 * @return Sticking or Sliding
	 */
	Type getTypeLinear() const;

	/**
	 * @brief Get the angular type of contact.
	 * @return Sticking or Sliding
	 */
	Type getTypeAngular() const;

	//! @brief Change type of contact to leaving.
	void setTypeLeaving();

	/**
	 * @brief Set the type of the contact.
	 * @param linear [in] the linear type (sticking/sliding)
	 * @param angular [in] the angular type (sticking/sliding)
	 */
	void setType(Type linear, Type angular);

	/**
	 * @brief Get the contact currently used.
	 * @return a reference to the contact.
	 */
	const rwsim::contacts::Contact& getContact() const;

	/**
	 * @brief Update with a new rwsim::contacts::Contact.
	 * @param contact [in] the new contact.
	 * @param state [in] the state where the contact was detected.
	 */
	void setContact(const rwsim::contacts::Contact &contact, const rw::kinematics::State &state);

	//! @copydoc TNTConstraint::update
	virtual void update(TNTIslandState &tntstate, const rw::kinematics::State &rwstate);

	//! @copydoc TNTConstraint::reset
	virtual void reset(TNTIslandState &tntstate, const rw::kinematics::State &rwstate);

	//! @copydoc TNTConstraint::getConstraintModes
	virtual std::vector<Mode> getConstraintModes() const;

	//! @copydoc TNTConstraint::getDimVelocity
	virtual std::size_t getDimVelocity() const;

	//! @copydoc TNTConstraint::getDimWrench
	virtual std::size_t getDimWrench() const;

	/**
	 * @brief Get the normal of the contact in world coordinates.
	 * @param tntstate [in] the current state.
	 * @return the normal.
	 */
	rw::math::Vector3D<> getNormalW(const TNTIslandState &tntstate) const;

private:
	bool _leaving;
	Type _linearType;
	Type _angularType;
	rwsim::contacts::Contact _contact;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_ */