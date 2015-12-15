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

#include "SimulatorStatistics.hpp"

#include "LogValues.hpp"
#include "SimulatorLogScope.hpp"
#include "SimulatorLogEntry.hpp"

using namespace rwsim::log;

SimulatorStatistics::SimulatorStatistics(const SimulatorLogScope* log):
	_log(log)
{
}

SimulatorStatistics::~SimulatorStatistics() {
}

void SimulatorStatistics::update() {
	_singleValues.clear();
	_multipleValues.clear();

	// Collect all data from children (of type SimulatorLogValues or SimulatorLogScope)
	DataSeries data;
	const std::vector<SimulatorLog::Ptr> children = _log->getChildren();
	BOOST_FOREACH(const SimulatorLog::Ptr child, children) {
		if (const SimulatorLogScope::Ptr scope = child.cast<SimulatorLogScope>()) {
			const rw::common::Ptr<const SimulatorStatistics> stats = scope->getStatistics();
			if (!stats.isNull()) {
				const DataSeries& prop = stats->getPropagated();
				for (DataSeries::const_iterator it = prop.begin(); it != prop.end(); it++) {
					BOOST_FOREACH(double val, it->second) {
						data[it->first].push_back(val);
					}
				}
			}
		} else if (child.cast<LogValues>()) {
			const rw::common::Ptr<const LogValues> values = child.cast<const LogValues>();
			const std::vector<double>& val = values->getValues();
			const std::vector<std::string>& labels = values->getLabels();
			RW_ASSERT(val.size() == labels.size());
			for (std::size_t i = 0; i < val.size(); i++)
				data[labels[i]].push_back(val[i]);
		}
	}

	// Now determine if the values in data map should be propagated to higher levels, or if
	// it belongs to this level.
	for (DataSeries::iterator it = data.begin(); it != data.end(); it++) {
		if (it->second.size() > 1) {
			BOOST_FOREACH(double val, it->second) {
				_multipleValues[it->first].push_back(val);
			}
		} else {
			_singleValues[it->first].push_back(it->second[0]);
		}
	}
}

bool SimulatorStatistics::hasData() const {
	return _multipleValues.size() > 0 || _singleValues.size() > 0;
}

const SimulatorStatistics::DataSeries& SimulatorStatistics::getSeries() const {
	return _multipleValues;
}

const SimulatorStatistics::DataSeries& SimulatorStatistics::getPropagated() const {
	return _singleValues;
}
