/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_LOG_LOGCONSTRAINTFORCETORQUE_HPP_
#define RWSIM_LOG_LOGCONSTRAINTFORCETORQUE_HPP_

/**
 * @file LogConstraintForceTorque.hpp
 *
 * \copydoc rwsim::log::LogConstraintForceTorque
 */

#include "LogForceTorque.hpp"

namespace rwsim {
namespace log {

class LogConstraints;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log wrench for constraints.
 */
class LogConstraintForceTorque: public LogForceTorque {
public:
    //! Smart pointer type of LogConstraintForceTorque
    typedef rw::common::Ptr<LogConstraintForceTorque> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogConstraintForceTorque(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogConstraintForceTorque();

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

	//! @copydoc LogForceTorque::createNew
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
	 * @brief Get the positions of the constraints.
	 *
	 * This is similar to getLinkedEntries.
	 *
	 * @return the log entry with positions of constraints (or NULL if not linked).
	 */
	rw::common::Ptr<LogConstraints> getConstraints() const;

private:
	rw::common::Ptr<LogConstraints> _constraints;
	const std::string _emptyStr;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCONSTRAINTFORCETORQUE_HPP_ */
