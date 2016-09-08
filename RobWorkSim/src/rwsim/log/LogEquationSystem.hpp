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

#ifndef RWSIM_LOG_LOGEQUATIONSYSTEM_HPP_
#define RWSIM_LOG_LOGEQUATIONSYSTEM_HPP_

/**
 * @file LogEquationSystem.hpp
 *
 * \copydoc rwsim::log::LogEquationSystem
 */

#include "SimulatorLogEntry.hpp"

#include <Eigen/Core>

namespace rwsim {
namespace log {

class LogContactSet;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log entry for a linear equation system \f$\mathbf{A}\mathbf{x}=\mathbf{b}\f$.
 */
class LogEquationSystem: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogEquationSystem
    typedef rw::common::Ptr<LogEquationSystem> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogEquationSystem(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogEquationSystem();

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
	 * @brief Set the equation system.
	 * @param A [in] the matrix.
	 * @param b [in] the right-hand-side.
	 */
	void set(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

	/**
	 * @brief Set the solution to the system.
	 * @param x [in] the solution.
	 */
	void setSolution(const Eigen::VectorXd& x);

	/**
	 * @brief Get the matrix for the linear equation system.
	 * @return a reference to the matrix.
	 */
	const Eigen::MatrixXd& A() const;

	/**
	 * @brief Get the right-hand side of the equation system.
	 * @return a reference to the right-hand side.
	 */
	const Eigen::VectorXd& b() const;

	/**
	 * @brief Get the solution.
	 * @return a reference to the solution.
	 */
	const Eigen::VectorXd& x() const;

private:
	Eigen::MatrixXd _A;
	Eigen::VectorXd _x;
	Eigen::VectorXd _b;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGEQUATIONSYSTEM_HPP_ */
