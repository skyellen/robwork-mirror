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

#ifndef RWSIM_LOG_LOGCONTACTFORCETORQUE_HPP_
#define RWSIM_LOG_LOGCONTACTFORCETORQUE_HPP_

/**
 * @file LogContactForceTorque.hpp
 *
 * \copydoc rwsim::log::LogContactForceTorque
 */

#include "LogForceTorque.hpp"

namespace rwsim {
namespace log {

class LogContactSet;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log a set of contact forces and torques.
 */
class LogContactForceTorque: public LogForceTorque {
public:
    //! Smart pointer type of LogContactForceTorque
    typedef rw::common::Ptr<LogContactForceTorque> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogContactForceTorque(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogContactForceTorque();

	//! @copydoc SimulatorLogEntry::getType
	virtual std::string getType() const;

	//! @copydoc LogForceTorque::operator==
	virtual bool operator==(const SimulatorLog &b) const;

	//! @copydoc SimulatorLogEntry::getLinkedEntries
	virtual std::list<SimulatorLogEntry::Ptr> getLinkedEntries() const;

	//! @copydoc SimulatorLogEntry::autoLink
	virtual bool autoLink();

	//! @copydoc SimulatorLogEntry::createNew
	virtual SimulatorLogEntry::Ptr createNew(SimulatorLogScope* parent) const;

	//! @copydoc LogForceTorque::sizeLinkedEntry
	virtual int sizeLinkedEntry() const;

	//! @copydoc LogForceTorque::getNameA
	virtual const std::string& getNameA(std::size_t i) const;

	//! @copydoc LogForceTorque::getNameB
	virtual const std::string& getNameB(std::size_t i) const;

	//! @copydoc LogForceTorque::getPositionA
	virtual rw::math::Vector3D<> getPositionA(std::size_t i) const;

	//! @copydoc LogForceTorque::getPositionB
	virtual rw::math::Vector3D<> getPositionB(std::size_t i) const;

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

private:
	rw::common::Ptr<LogContactSet> _contacts;
	const std::string _emptyStr;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCONTACTFORCETORQUE_HPP_ */
