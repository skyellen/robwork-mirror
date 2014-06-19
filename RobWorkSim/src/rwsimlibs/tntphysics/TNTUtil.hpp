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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTUTIL_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTUTIL_HPP_

/**
 * @file TNTUtil.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTUtil
 */

#include <vector>
#include <set>

#include <rw/math/Vector3D.hpp>
#include <rwsim/contacts/ContactStrategyTracking.hpp>

// Forward declarations
namespace rw { namespace kinematics { class Frame; } };
namespace rw { namespace kinematics { class State; } };
namespace rwsim { namespace contacts { class Contact; } };
namespace rwsim { namespace contacts { class ContactDetectorTracking; } };

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBodyConstraintManager;
class TNTIslandState;
class TNTContact;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Utility functions for the TNT physics engine.
 *
 * Most functions are related to contact tracking.
 * Contacts can in general be in two states, either marked or known.
 *
 * Marked contacts are contacts that belongs to a specific class of contacts.
 * The marked contacts can not need be tracked individually, but only as a group.
 *
 * All new contacts detected by the ContactDetector will be marked MARK_RAW.
 *
 * A predefined mark, MARK_NEW, is defined for contacts that are candidates for
 * being treated as bouncing. This is used mainly during rollback to limit the effort
 * used on contact detection.
 *
 * Finally known contacts are contacts that has a TNTContact attached as user data.
 * This allows tracking existing known contacts to update the contact positions.
 *
 * Besides the contact tracking functions, there are some helper functions defined
 * with the purpose of making the main simulation loop easier to read.
 */
class TNTUtil {
public:
	//! @brief Tracking structure that utilizes the features of the contact detector with tracking.
	struct TNTUserData: public rwsim::contacts::ContactStrategyTracking::UserData {
		/**
		 * @brief Construct new data for the given TNTContact.
		 * @param contact [in] the contact to assign to this user-data structure.
		 */
		TNTUserData(const TNTContact* contact): contact(contact) {}

		//! @brief The TNTContact assigned to the given contact.
		const TNTContact* const contact;

		static rwsim::contacts::ContactStrategyTracking::UserData::Ptr make(const TNTContact* contact) {
			return rw::common::ownedPtr(new TNTUserData(contact));
		}
	};

	//! @brief Predefined mark that can be used to mark new contacts.
	static const rwsim::contacts::ContactStrategyTracking::UserData::Ptr MARK_NEW;

	//! @brief Predefined mark that can is used for newly detected contacts.
	static const rwsim::contacts::ContactStrategyTracking::UserData::Ptr MARK_RAW;

	/**
	 * @brief Find the contacts with the given mark.
	 * @param input [in] the list of contacts to classify.
	 * @param tracking [in] the tracking information from the rwsim::contacts::ContactDetector.
	 * @param mark [in] the mark to search for.
	 * @return a list of indices for contacts that has the given mark.
	 */
	static std::vector<std::size_t> getMarkedContacts(const std::vector<rwsim::contacts::Contact>& input, const rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Get the contacts refered to by a list of indices.
	 * @param contacts [in] the contacts.
	 * @param ids [in] the indices.
	 * @return new list with only the contacts in the id list
	 */
	static std::vector<rwsim::contacts::Contact> getContacts(const std::vector<rwsim::contacts::Contact>& contacts, const std::vector<std::size_t>& ids);

	/**
	 * @brief Remove the new contacts that are not in penetration and leave the new penetrating contacts.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void removeNonPenetrating(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Remove the new contacts that are in penetration and leave the new non-penetrating contacts.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void removePenetrating(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Remove all known contacts, leaving only the marked ones.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param bc [in] the body-constraint manager that has the constraints in the system.
	 * @param tntstate [in] the state.
	 */
	static void removeKnown(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate);

	/**
	 * @brief Remove all contacts with the given mark.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param mark [in] only remove contacts that has this mark.
	 */
	static void remove(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Mark all contacts with a given mark with a different mark.
	 * @param contacts [in/out] the complete contact list.
	 * @param tracking [in/out] the tracking information.
	 * @param oldMark [in] the old mark.
	 * @param newMark [in] the new mark.
	 */
	static void mark(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, rwsim::contacts::ContactStrategyTracking::UserData::Ptr oldMark, rwsim::contacts::ContactStrategyTracking::UserData::Ptr newMark);

	/**
	 * @brief Do integration for all bodies and update the state structures.
	 * @param dt [in] the stepsize.
	 * @param gravity [in] the gravity in world coordinates.
	 * @param bc [in] the body-constraint manager.
	 * @param tntstate [in/out] the current state to update.
	 * @param rwstate [in/out] the current state to update.
	 */
	static void step(double dt, const rw::math::Vector3D<>& gravity, const TNTBodyConstraintManager* bc, TNTIslandState& tntstate, rw::kinematics::State& rwstate);

	/**
	 * @brief Get the minimum distance in a list of contacts.
	 * @param contacts [in] the list of contacts.
	 * @return the minimum distance.
	 */
	static double minDistance(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Get the maximum distance in a list of contacts.
	 * @param contacts [in] the list of contacts.
	 * @return the maximum distance.
	 */
	static double maxDistance(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Remove contacts with specific mark that has a distance greater than some threshold.
	 * @param contacts [in/out] the list of contacts.
	 * @param tracking [in/out] the tracking information.
	 * @param threshold [in] the threshold.
	 * @param mark [in] only the contacts with this mark can be removed.
	 */
	static void removeContactsOutsideThreshold(std::vector<rwsim::contacts::Contact>& contacts, rwsim::contacts::ContactDetectorTracking& tracking, double threshold, rwsim::contacts::ContactStrategyTracking::UserData::Ptr mark);

	/**
	 * @brief Update the temporary contacts with new contact information.
	 * @param contacts [in] the updated contact information.
	 * @param tracking [in] the tracking information.
	 * @param bc [in] the body-constraint manager.
	 * @param tntstate [in] the current state to update with new information.
	 * @param rwstate [in] the current state.
	 */
	static void updateTemporaryContacts(const std::vector<rwsim::contacts::Contact>& contacts, const rwsim::contacts::ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, TNTIslandState& tntstate, const rw::kinematics::State &rwstate);

private:
	static rwsim::contacts::ContactStrategyTracking::UserData::Ptr getNewMark();
	class DummyMark: public rwsim::contacts::ContactStrategyTracking::UserData {};
	TNTUtil() {};
	virtual ~TNTUtil() {};
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTUTIL_HPP_ */
