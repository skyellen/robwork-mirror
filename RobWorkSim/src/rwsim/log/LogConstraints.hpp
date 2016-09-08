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

#ifndef RWSIM_LOG_LOGCONSTRAINTS_HPP_
#define RWSIM_LOG_LOGCONSTRAINTS_HPP_

/**
 * @file LogConstraints.hpp
 *
 * \copydoc rwsim::log::LogConstraints
 */

#include "SimulatorLogEntry.hpp"

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log type for constraints.
 */
class LogConstraints: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogConstraints
    typedef rw::common::Ptr<LogConstraints> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogConstraints(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogConstraints();

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
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

	/**
	 * @brief Get the number of constraint currently in set.
	 * @return the number of constraint.
	 */
	std::size_t size() const;

	//! @brief Information for a constraint.
	struct Constraint {
		//! @brief Name of the parent frame.
		std::string frameA;
		//! @brief Name of the child frame.
		std::string frameB;
		//! @brief Textual description of the type of constraint.
		std::string type;
		//! @brief Position in world coordinates of the constraint on the parent.
		rw::math::Vector3D<> posA;
		//! @brief Position in world coordinates of the constraint on the child.
		rw::math::Vector3D<> posB;
		//! @brief The coordinate frames for the linear rotation constraint on the parent.
		rw::math::Rotation3D<> rotAlin;
		//! @brief The coordinate frames for the linear rotation constraint on the child.
		rw::math::Rotation3D<> rotBlin;
		//! @brief The coordinate frames for the angular rotation constraint on the parent.
		rw::math::Rotation3D<> rotAang;
		//! @brief The coordinate frames for the angular rotation constraint on the child.
		rw::math::Rotation3D<> rotBang;
		/**
		 * @brief Check if equal to other Constraint struct.
		 * @param b [in] other constraints.
		 * @return true if equal, false otherwise.
		 */
		bool operator==(const Constraint &b) const;
	};

	/**
	 * @brief Append a constraint.
	 * @param constraint [in] the constraint information to add.
	 */
	void addConstraint(const Constraint& constraint);

	/**
	 * @brief Get the constraints.
	 * @return a vector for constraint information.
	 */
	const std::vector<Constraint>& getConstraints() const;

	/**
	 * @brief Get a constraint.
	 * @param i [in] the index.
	 * @return the constraint information.
	 */
	const Constraint& getConstraint(std::size_t i) const;

private:
	std::vector<Constraint> _constraints;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCONSTRAINTS_HPP_ */
