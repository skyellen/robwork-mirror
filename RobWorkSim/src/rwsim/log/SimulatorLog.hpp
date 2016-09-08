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

#ifndef RWSIM_LOG_SIMULATORLOG_HPP_
#define RWSIM_LOG_SIMULATORLOG_HPP_

/**
 * @file SimulatorLog.hpp
 *
 * \copydoc rwsim::log::SimulatorLog
 */

#include <rw/common/Serializable.hpp>

namespace rwsim {
namespace log {
class SimulatorLogScope;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Base class for a hierarchical simulator log.
 *
 * Every log item should have a filename that refers to the file that added the log item,
 * as well as a description that is as short as possible.
 */
class SimulatorLog: public rw::common::Serializable {
public:
    //! Smart pointer type of SimulatorLog
    typedef rw::common::Ptr<SimulatorLog> Ptr;

    /**
     * @brief Constructor.
     * @note This item will not be added as a child to the parent scope using this constructor.
     * @param parent the parent of this log item.
     */
	SimulatorLog(SimulatorLogScope* parent = NULL);

	//! @brief Destructor.
	virtual ~SimulatorLog();

	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	/**
	 * @brief Get the parent of this log item.
	 * @return the parent (NULL if no parent).
	 */
	virtual SimulatorLogScope* getParent() const;

	/**
	 * @brief Get the number of children under this log item.
	 * @return the number of children.
	 */
	virtual std::size_t children() const = 0;

	/**
	 * @brief Get a textual representation of the type of the item.
	 * @return the type of item as a string.
	 */
	virtual std::string getType() const = 0;

	/**
	 * @brief Check if logs are identical.
	 * @param b [in] other log to compare with.
	 * @return true if identical, false otherwise.
	 */
	virtual bool operator==(const SimulatorLog &b) const;

	/**
	 * @brief Check if logs are non-identical.
	 * @param b [in] other log to compare with.
	 * @return true if not identical, false otherwise.
	 */
	virtual bool operator!=(const SimulatorLog &b) const;

	/**
	 * @brief Get the full filename with path for where this log entry was created.
	 * @return the full filename.
	 */
	virtual std::string getFilename() const;

	/**
	 * @brief Set the name of the file where this log entry was created.
	 * @param file [in] the filename (expected to have full path included).
	 */
	virtual void setFilename(const std::string& file);

	/**
	 * @brief Set the name of the file where this log entry was created.
	 * @param file [in] the filename (expected to have full path included).
	 */
	virtual void setFilename(const char* file);

	/**
	 * @brief Get a string describing this entry.
	 * @return the string.
	 */
	virtual std::string getDescription() const;

	/**
	 * @brief Set a very short description of this entry.
	 * @param description [in] the description to set.
	 */
	virtual void setDescription(const std::string& description);

private:
	SimulatorLogScope* const _parent;
	std::string _filename;
	std::string _description;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_SIMULATORLOG_HPP_ */
