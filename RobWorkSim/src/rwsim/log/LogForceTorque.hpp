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

#ifndef RWSIM_LOG_LOGFORCETORQUE_HPP_
#define RWSIM_LOG_LOGFORCETORQUE_HPP_

/**
 * @file LogForceTorque.hpp
 *
 * \copydoc rwsim::log::LogForceTorque
 */

#include "SimulatorLogEntry.hpp"
#include <rw/math/Wrench6D.hpp>

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Logging for forces and torques.
 */
class LogForceTorque: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogForceTorque
    typedef rw::common::Ptr<LogForceTorque> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogForceTorque(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogForceTorque();

    //! @copydoc SimulatorLogEntry::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

    //! @copydoc SimulatorLogEntry::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLogEntry::getType
	virtual std::string getType() const = 0;

	//! @copydoc SimulatorLogEntry::getLinkedEntries
	virtual std::list<SimulatorLogEntry::Ptr> getLinkedEntries() const = 0;

	//! @copydoc SimulatorLogEntry::autoLink
	virtual bool autoLink() = 0;

	//! @copydoc SimulatorLogEntry::createNew
	virtual SimulatorLogEntry::Ptr createNew(SimulatorLogScope* parent) const = 0;

	/**
	 * @brief Get the size of the linked entry (if any).
	 * @return size of the linked entry or negative value if not linked.
	 */
	virtual int sizeLinkedEntry() const = 0;

	/**
	 * @brief Get name of the first object.
	 * @param i [in] the index.
	 * @return the name or an empty string if not linked properly.
	 */
	virtual const std::string& getNameA(std::size_t i) const = 0;

	/**
	 * @brief Get name of the second object.
	 * @param i [in] the index.
	 * @return the name or an empty string if not linked properly.
	 */
	virtual const std::string& getNameB(std::size_t i) const = 0;

	/**
	 * @brief Get anchor position on the first object.
	 * @param i [in] the index.
	 * @return the anchor point or zero if not linked.
	 */
	virtual rw::math::Vector3D<> getPositionA(std::size_t i) const = 0;

	/**
	 * @brief Get anchor position on the second object.
	 * @param i [in] the index.
	 * @return the anchor point or zero if not linked.
	 */
	virtual rw::math::Vector3D<> getPositionB(std::size_t i) const = 0;

	/**
	 * @brief Get the contact wrench acting at the first body.
	 * @param i [in] the constraint to get wrench for.
	 * @return the wrench.
	 */
	rw::math::Wrench6D<> getWrenchBodyA(std::size_t i) const;

	/**
	 * @brief Get the contact wrench acting at the second body.
	 * @param i [in] the constraint to get wrench for.
	 * @return the wrench.
	 */
	rw::math::Wrench6D<> getWrenchBodyB(std::size_t i) const;

	/**
	 * @brief Set a given wrench pair.
	 * @param i [in] the constraint to set wrench for.
	 * @param wrenchA [in] the wrench acting in the constraint at the first body.
	 * @param wrenchB [in] the wrench acting in the constraint at the second body.
	 */
	void setWrench(std::size_t i, const rw::math::Wrench6D<>& wrenchA, const rw::math::Wrench6D<>& wrenchB);

private:
	std::vector<std::pair<rw::math::Wrench6D<>, rw::math::Wrench6D<> > > _forces;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGFORCETORQUE_HPP_ */
