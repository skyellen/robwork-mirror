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

#ifndef RWSIM_LOG_LOGMESSAGE_HPP_
#define RWSIM_LOG_LOGMESSAGE_HPP_

/**
 * @file LogMessage.hpp
 *
 * \copydoc rwsim::log::LogMessage
 */

#include "SimulatorLogEntry.hpp"

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Logging of a generic message.
 */
class LogMessage: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogMessage
    typedef rw::common::Ptr<LogMessage> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogMessage(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogMessage();

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
	 * @brief Get the stream to output to.
	 * @return the stream.
	 */
	std::ostream& stream();

	/**
	 * @brief Get the message.
	 * @return the message.
	 */
	std::string getMessage() const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

private:
	std::stringstream _message;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGMESSAGE_HPP_ */
