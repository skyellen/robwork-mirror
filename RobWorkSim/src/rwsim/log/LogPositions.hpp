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

#ifndef RWSIM_LOG_LOGPOSITIONS_HPP_
#define RWSIM_LOG_LOGPOSITIONS_HPP_

/**
 * @file LogPositions.hpp
 *
 * \copydoc rwsim::log::LogPositions
 */

#include "SimulatorLogEntry.hpp"

#include <rw/math/Transform3D.hpp>

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Logging of body positions.
 */
class LogPositions: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogPositions
    typedef rw::common::Ptr<LogPositions> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogPositions(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogPositions();

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
	 * @brief Get the position map.
	 * @return the position map.
	 */
	const std::map<std::string, rw::math::Transform3D<> >& getPositions() const;

	/**
	 * @brief Get a position.
	 * @param name [in] the name of the body to get position for.
	 * @return the position as a transform.
	 */
	const rw::math::Transform3D<>& getPosition(const std::string& name) const;

	/**
	 * @brief Check if a position with the given name exists.
	 * @param name [in] the name to look for.
	 * @return true if found, false if not.
	 */
	bool has(const std::string& name) const;

	/**
	 * @brief Set the positions.
	 * @param positions [in] the positions.
	 */
	void setPositions(const std::map<std::string, rw::math::Transform3D<> >& positions);

	/**
	 * @brief Get the number of positions stored.
	 * @return the number of positions.
	 */
	std::size_t size() const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

private:
	std::map<std::string,rw::math::Transform3D<> > _positions;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGPOSITIONS_HPP_ */
