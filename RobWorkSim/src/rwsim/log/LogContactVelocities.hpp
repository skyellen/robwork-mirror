/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_LOG_LOGCONTACTVELOCITIES_HPP_
#define RWSIM_LOG_LOGCONTACTVELOCITIES_HPP_

/**
 * @file LogContactVelocities.hpp
 *
 * \copydoc rwsim::log::LogContactVelocities
 */

#include "SimulatorLogEntry.hpp"

namespace rwsim {
namespace log {

class LogContactSet;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log a set of contact velocities.
 */
class LogContactVelocities: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogContactVelocities
    typedef rw::common::Ptr<LogContactVelocities> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogContactVelocities(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogContactVelocities();

    //! @copydoc SimulatorLogEntry::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

    //! @copydoc SimulatorLogEntry::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLogEntry::getType
	virtual std::string getType() const;

	//! @copydoc SimulatorLogEntry::operator==
	virtual bool operator==(const SimulatorLog &b) const;

	//! @copydoc SimulatorLogEntry::getLinkedEntries
	virtual std::list<SimulatorLogEntry::Ptr> getLinkedEntries() const;

	//! @copydoc SimulatorLogEntry::autoLink
	virtual bool autoLink();

	//! @copydoc SimulatorLogEntry::createNew
	virtual SimulatorLogEntry::Ptr createNew(SimulatorLogScope* parent) const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

	/**
	 * @brief Get the positions of the contacts.
	 *
	 * This is similar to getLinkedEntries.
	 *
	 * @return the log entry with positions of contacts (or NULL if not linked).
	 */
	rw::common::Ptr<LogContactSet> getContacts() const;

	/**
	 * @brief Get the contact velocity acting at the first body.
	 * @param i [in] the contact to get velocity for.
	 * @return the velocity.
	 */
	rw::math::Vector3D<> getVelocityBodyA(std::size_t i) const;

	/**
	 * @brief Get the contact velocity acting at the second body.
	 * @param i [in] the contact to get velocity for.
	 * @return the velocity.
	 */
	rw::math::Vector3D<> getVelocityBodyB(std::size_t i) const;

	/**
	 * @brief Set a given velocity pair.
	 * @param i [in] the contact to set velocity for.
	 * @param velocityA [in] the velocity acting in the contact at the first body.
	 * @param velocityB [in] the velocity acting in the contact at the second body.
	 */
	void setVelocity(std::size_t i, const rw::math::Vector3D<>& velocityA, const rw::math::Vector3D<>& velocityB);

private:
	rw::common::Ptr<LogContactSet> _contacts;
	std::vector<std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > > _velocities;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCONTACTVELOCITIES_HPP_ */
