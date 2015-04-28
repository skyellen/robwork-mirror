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

class TNTFrictionModel;
class TNTFrictionModelData;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief A special form of constraint that changes continuously during simulation.
 */
class TNTContact: public TNTConstraint {
public:
	//! @brief Contact friction types
	typedef enum Type {
		None = 0,    //!< None - no friction is applied, and the contact can move freely.
		Sticking = 1,//!< Sticking - the relative velocity is restricted to zero.
		Sliding = 2  //!< Sliding - force will be added opposite to the relative velocity.
	} Type;

	/**
	 * @brief Construct new unspecified contact between two bodies.
	 * @param parent [in] first body.
	 * @param child [in] second body.
	 * @param friction [in] the friction model to use.
	 */
	TNTContact(const TNTBody* parent, const TNTBody* child, const TNTFrictionModel& friction);

	/**
	 * @brief Construct new specified contact between two bodies.
	 * @param parent [in] first body.
	 * @param child [in] second body.
	 * @param friction [in] the friction model to use.
	 * @param contact [in] the contact.
	 * @param state [in] the state where the contact was found.
	 */
	TNTContact(const TNTBody* parent, const TNTBody* child, const TNTFrictionModel& friction, const rwsim::contacts::Contact &contact, const rw::kinematics::State &state);

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

	//! @copydoc TNTConstraint::step
	virtual void step(TNTIslandState &tntstate, const rw::kinematics::State &rwstate, double h);

	//! @copydoc TNTConstraint::getConstraintModes
	virtual std::vector<Mode> getConstraintModes() const;

	//! @copydoc TNTConstraint::getDimVelocity
	virtual std::size_t getDimVelocity() const;

	//! @copydoc TNTConstraint::getDimWrench
	virtual std::size_t getDimWrench() const;

	//! @copydoc TNTConstraint::getDimFree
	virtual std::size_t getDimFree() const;

	//! @copydoc TNTConstraint::getLinearRotationParentForceW
	virtual rw::math::Rotation3D<> getLinearRotationParentForceW(const TNTIslandState &state) const;

	//! @copydoc TNTConstraint::getLinearRotationChildForceW
	virtual rw::math::Rotation3D<> getLinearRotationChildForceW(const TNTIslandState &state) const;

	/**
	 * @brief Get the normal of the contact in world coordinates.
	 * @param tntstate [in] the current state.
	 * @return the normal.
	 */
	rw::math::Vector3D<> getNormalW(const TNTIslandState &tntstate) const;

	/**
	 * @brief Get the current tangential friction dir.
	 * @param tntstate [in] the current state.
	 * @return the direction.
	 */
	rw::math::Vector3D<> getFrictionDirW(const TNTIslandState &tntstate) const;

	/**
	 * @brief Get a string representation of the contact Type.
	 * @param type [in] the type.
	 * @return the type as a string.
	 */
	static std::string toString(Type type);

private:
	bool _leaving;
	Type _linearType;
	Type _angularType;
	rwsim::contacts::Contact _contact;
	const TNTFrictionModel& _friction;
	rw::math::Rotation3D<> _offsetParent;
	rw::math::Rotation3D<> _offsetChild;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_ */
