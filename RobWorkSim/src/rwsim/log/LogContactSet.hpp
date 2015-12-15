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

#ifndef RWSIM_LOG_LOGCONTACTSET_HPP_
#define RWSIM_LOG_LOGCONTACTSET_HPP_

/**
 * @file LogContactSet.hpp
 *
 * \copydoc rwsim::log::LogContactSet
 */

#include "SimulatorLogEntry.hpp"

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log a set of contacts.
 */
class LogContactSet: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogContactSet
    typedef rw::common::Ptr<LogContactSet> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
	LogContactSet(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogContactSet();

    //! @copydoc SimulatorLogEntry::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

    //! @copydoc SimulatorLogEntry::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLogEntry::getType
	virtual std::string getType() const;

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
	 * @brief Get the list of contacts.
	 * @return the list of contacts.
	 */
	const std::vector<rwsim::contacts::Contact>& getContacts() const;

	/**
	 * @brief Get a contact.
	 * @param i [in] the index.
	 * @return the contact.
	 */
	const rwsim::contacts::Contact& getContact(std::size_t i) const;

	/**
	 * @brief Set a new list of contacts.
	 * @param contacts [in] the list of contacts.
	 */
	void setContacts(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Append a contact.
	 * @param contact [in] the contact to append.
	 */
	void addContact(const rwsim::contacts::Contact& contact);

	/**
	 * @brief Get the number of contacts currently in set.
	 * @return the number of contacts.
	 */
	std::size_t size() const;

private:
	std::vector<rwsim::contacts::Contact> _contacts;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCONTACTSET_HPP_ */
