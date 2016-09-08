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

#ifndef RWSIM_LOG_SIMULATORLOGSCOPE_HPP_
#define RWSIM_LOG_SIMULATORLOGSCOPE_HPP_

/**
 * @file SimulatorLogScope.hpp
 *
 * \copydoc rwsim::log::SimulatorLogScope
 */

#include "SimulatorLog.hpp"

namespace rwsim {
namespace log {

class SimulatorStatistics;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief A scope can have children, and the type allows hierarchical logging.
 *
 * A scope can have one SimulatorStatistics object attached to it. SimulatorStatistics
 * is a special method for doing statistics on a SimulatorLog. Please see this class for
 * information about how this is done.
 */
class SimulatorLogScope: public SimulatorLog {
public:
    //! Smart pointer type of SimulatorLogScope
    typedef rw::common::Ptr<SimulatorLogScope> Ptr;

    //! @copydoc SimulatorLog::SimulatorLog
	SimulatorLogScope(SimulatorLogScope* parent = NULL);

	//! @brief Destructor.
	virtual ~SimulatorLogScope();

	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLog::children
	virtual std::size_t children() const;

	//! @copydoc SimulatorLog::getType
	virtual std::string getType() const;

	//! @copydoc SimulatorLog::operator==
	virtual bool operator==(const SimulatorLog &b) const;

	/**
	 * @brief Get the children of this scope.
	 * @return a vector of children log items.
	 */
	std::vector<SimulatorLog::Ptr> getChildren() const;

	/**
	 * @brief Get a specific child item.
	 * @param id [in] the id of the child.
	 * @return the child item (or NULL if not found).
	 */
	SimulatorLog::Ptr getChild(std::size_t id) const;

	/**
	 * @brief Get the id of a specific log item that is known to be a child.
	 * @param child [in] the child item to search for.
	 * @return the index of the child (or the number of children if not found).
	 */
	std::size_t indexOf(const SimulatorLog* child) const;

	/**
	 * @brief Add a child to this scope.
	 * @param child the child to add.
	 */
	void appendChild(SimulatorLog::Ptr child);

	/**
	 * @brief Get statistics for this scope.
	 * @return the statistics.
	 */
	rw::common::Ptr<const SimulatorStatistics> getStatistics();

	/**
	 * @brief Get the line in the file where this scope begins.
	 * @return the first line.
	 */
	virtual int lineBegin() const;

	/**
	 * @brief Get the line in the file where this scope ends.
	 * @return the last line.
	 */
	virtual int lineEnd() const;

	/**
	 * @brief Set the line in the file where this scope begins.
	 * @param line [in] the first line.
	 */
	virtual void setLineBegin(int line);

	/**
	 * @brief Set the line in the file where this scope ends.
	 * @param line [in] the last line.
	 */
	virtual void setLineEnd(int line);

private:
	std::vector<SimulatorLog::Ptr> _children;
	rw::common::Ptr<SimulatorStatistics> _statistics;
	std::pair<int, int> _line;
};

//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_SIMULATORLOGSCOPE_HPP_ */
