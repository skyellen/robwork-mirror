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

#ifndef RWSIM_LOG_SIMULATORLOGENTRY_HPP_
#define RWSIM_LOG_SIMULATORLOGENTRY_HPP_

/**
 * @file SimulatorLogEntry.hpp
 *
 * \copydoc rwsim::log::SimulatorLogEntry
 */

#include "SimulatorLog.hpp"

#include <rw/common/ExtensionPoint.hpp>

namespace rwsim {
namespace log {

class SimulatorLogScope;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief A leaf log item with no children.
 *
 * Leaf items can use a linking feature, to avoid too much redundant information.
 * Linking should always be done using the autoLink function, which will search
 * backwards in the current scope for a correct entry type to link with. If none
 * is found, parent scopes are searched in the same way until the base scope is
 * reached.
 *
 * If for instance a velocity is to be logged, there should be a relevant position
 * log entry previous to this velocity entry. This is important for visualization
 * purposes, to know where vectors act.
 */
class SimulatorLogEntry: public SimulatorLog {
public:
    //! Smart pointer type of SimulatorLogEntry
    typedef rw::common::Ptr<SimulatorLogEntry> Ptr;

    //! @copydoc SimulatorLog::SimulatorLog
	SimulatorLogEntry(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~SimulatorLogEntry();

	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLog::children
	virtual std::size_t children() const;

	//! @copydoc SimulatorLog::operator==
	virtual bool operator==(const SimulatorLog &b) const;

	/**
	 * @brief Get the line number in the file where this entry was added.
	 * @return the line number.
	 */
	virtual int line() const;

	/**
	 * @brief Set the line number where this entry was added.
	 * @param line [in] the line number.
	 */
	virtual void setLine(int line);

	//! @copydoc SimulatorLog::getType
	virtual std::string getType() const = 0;

	/**
	 * @brief Get a list of other entries that this entry is linked to.
	 *
	 * Using linked entries makes it possible to avoid logging too much redundant information.
	 * @return a list of linked entries.
	 */
	virtual std::list<SimulatorLogEntry::Ptr> getLinkedEntries() const = 0;

	/**
	 * @brief Do automatic linking to other entries.
	 * @return true if linking succeeded, false otherwise.
	 */
	virtual bool autoLink() = 0;

	/**
	 * @brief Create a new entry of the same type (no data is copied).
	 * @param parent the parent of the new entry.
	 * @return the new entry.
	 */
	virtual SimulatorLogEntry::Ptr createNew(SimulatorLogScope* parent) const = 0;

	/**
	 * @addtogroup extensionpoints
	 * @extensionpoint{rwsim::log::SimulatorLogEntry::Factory,rwsim::log::SimulatorLogEntry,rwsim.log.SimulatorLogEntry}
	 */

	/**
	 * @brief A factory for a SimulatorLogEntry. This factory also defines an
	 * extension point for SimulatorLogEntry.
	 */
    class Factory: public rw::common::ExtensionPoint<SimulatorLogEntry> {
    public:
    	/**
    	 * @brief Get the available entry types.
    	 * @return a vector of entry types.
    	 */
    	static std::vector<std::string> getEntryTypes();

    	/**
    	 * @brief Check if entry type is available.
    	 * @param entryType [in] the name of the entry type.
    	 * @return true if available, false otherwise.
    	 */
    	static bool hasEntryType(const std::string& entryType);

    	/**
    	 * @brief Create a new entry.
    	 * @param entryType [in] the type of entry.
    	 * @param parent [in] the parent scope.
    	 * @return a pointer to a new entry.
    	 */
    	static SimulatorLogEntry::Ptr makeEntry(const std::string& entryType, SimulatorLogScope* parent);

    private:
        Factory();
    };

private:
	int _line;
};

//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_SIMULATORLOGENTRY_HPP_ */
