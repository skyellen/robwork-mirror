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

#ifndef RWSIM_LOG_SIMULATORSTATISTICS_HPP_
#define RWSIM_LOG_SIMULATORSTATISTICS_HPP_

/**
 * @file SimulatorStatistics.hpp
 *
 * \copydoc rwsim::log::SimulatorStatistics
 */

#include <rw/common/Ptr.hpp>

#include <vector>
#include <map>

namespace rwsim {
namespace log {

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Statistics utility for automatic generation of data set based on a SimulatorLogScope.
 */
class SimulatorStatistics {
public:
    //! Smart pointer type of SimulatorStatistics
    typedef rw::common::Ptr<SimulatorStatistics> Ptr;

    //! Type of a DataSeries collection (each series is a name and a list of numbers)
	typedef std::map<std::string, std::vector<double> > DataSeries;

	/**
	 * @brief Create a new statistics on the given SimulatorLogScope.
	 * @note Please use the SimulatorLogScope::getStatistics function instead.
	 * @note All children log scopes will have their statistics updated or added.
	 * @param log [in/out] the log to create statistics based on.
	 */
	SimulatorStatistics(const class SimulatorLogScope* log);

	//! @brief Destructor.
	virtual ~SimulatorStatistics();

	//! @brief Generate statistics.
	void update();

	//! @brief Check if any statistics was generated.
	bool hasData() const;

	/**
	 * @brief Get the found statistics.
	 * @return a collection of data series.
	 */
	const DataSeries& getSeries() const;

private:
	const DataSeries& getPropagated() const;

private:
	const class SimulatorLogScope* _log;
	DataSeries _singleValues;
	DataSeries _multipleValues;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_SIMULATORSTATISTICS_HPP_ */
