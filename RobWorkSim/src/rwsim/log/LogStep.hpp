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

#ifndef RWSIM_LOG_LOGSTEP_HPP_
#define RWSIM_LOG_LOGSTEP_HPP_

/**
 * @file LogStep.hpp
 *
 * \copydoc rwsim::log::LogStep
 */

#include "SimulatorLogScope.hpp"

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief A special type of scope that is also a simulation step.
 *
 * This scope saves additional information about the time step taken.
 */
class LogStep: public SimulatorLogScope {
public:
    //! Smart pointer type of LogStep
    typedef rw::common::Ptr<LogStep> Ptr;

    //! @copydoc SimulatorLog::SimulatorLog
	LogStep(SimulatorLogScope* parent);

	//! @brief Destructor
	virtual ~LogStep();

	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLog::getType
	virtual std::string getType() const;

	//! @copydoc SimulatorLogScope::getDescription
	virtual std::string getDescription() const;

	/**
	 * @brief Get the time at the beginning of timestep.
	 * @return the initial time.
	 */
	virtual double timeBegin() const;

	/**
	 * @brief Get the time at the end of timestep.
	 * @return the final time.
	 */
	virtual double timeEnd() const;

	/**
	 * @brief Set the time at the beginning of timestep.
	 * @param time [in] the initial time.
	 */
	virtual void setTimeBegin(double time);

	/**
	 * @brief Set the time at the end of timestep.
	 * @param time [in] the final time.
	 */
	virtual void setTimeEnd(double time);

private:
	std::pair<double,double> _interval;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGSTEP_HPP_ */
