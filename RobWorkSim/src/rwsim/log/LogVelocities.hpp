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

#ifndef RWSIM_LOG_LOGVELOCITIES_HPP_
#define RWSIM_LOG_LOGVELOCITIES_HPP_

/**
 * @file LogVelocities.hpp
 *
 * \copydoc rwsim::log::LogVelocities
 */

#include "SimulatorLogEntry.hpp"

#include <rw/math/VelocityScrew6D.hpp>

namespace rwsim {
namespace log {

class LogPositions;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Logging of body velocities.
 */
class LogVelocities: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogVelocities
    typedef rw::common::Ptr<LogVelocities> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogVelocities(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogVelocities();

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
	 * @brief Get the positions of the bodies.
	 *
	 * This is similar to getLinkedEntries.
	 *
	 * @return the log entry with positions of bodies (or NULL if not linked).
	 */
	rw::common::Ptr<LogPositions> getPositions() const;

	/**
	 * @brief Get the velocity map.
	 * @return the velocity map.
	 */
	const std::map<std::string, rw::math::VelocityScrew6D<> >& getVelocities() const;

	/**
	 * @brief Get a velocity.
	 * @param name [in] the name of the body to get velocity for.
	 * @return the velocity.
	 */
	const rw::math::VelocityScrew6D<>& getVelocity(const std::string& name) const;

	/**
	 * @brief Check if a velocity with the given name exists.
	 * @param name [in] the name to look for.
	 * @return true if found, false if not.
	 */
	bool has(const std::string& name) const;

	/**
	 * @brief Set the velocities.
	 * @param velocities [in] the velocities.
	 */
	void setVelocities(const std::map<std::string, rw::math::VelocityScrew6D<> >& velocities);

	/**
	 * @brief Set a velocity.
	 * @param name [in] the name of the body to set velocity for.
	 * @param velocity [in] the velocity.
	 */
	void setVelocity(const std::string& name, const rw::math::VelocityScrew6D<>& velocity);

	/**
	 * @brief Get the number of velocities stored.
	 * @return the number of velocities.
	 */
	std::size_t size() const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

private:
	rw::common::Ptr<LogPositions> _positions;
	std::map<std::string,rw::math::VelocityScrew6D<> > _velocities;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGVELOCITIES_HPP_ */
