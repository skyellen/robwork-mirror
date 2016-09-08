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

#ifndef RWSIM_LOG_LOGVALUES_HPP_
#define RWSIM_LOG_LOGVALUES_HPP_

/**
 * @file LogValues.hpp
 *
 * \copydoc rwsim::log::LogValues
 */

#include "SimulatorLogEntry.hpp"

namespace rwsim {
namespace log {
//! @addtogroup rwsim_log

//! @{
/**
 * @brief Logging of numeric values. These values will also be used for SimulatorStatistics.
 */
class LogValues: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogValues
    typedef rw::common::Ptr<LogValues> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogValues(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogValues();

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
	 * @brief Set the values and labels (must be equal size).
	 * @param labels [in] the labels.
	 * @param values [in] the values.
	 */
	void setData(const std::vector<std::string>& labels, const std::vector<double>& values);

	/**
	 * @brief Get the number of values stored.
	 * @return the number of values.
	 */
	std::size_t size() const;

	/**
	 * @brief Get the labels.
	 * @return the labels.
	 */
	const std::vector<std::string>& getLabels() const;

	/**
	 * @brief Get the label for a given value.
	 * @param i [in] the index of the label and value.
	 * @return the label.
	 */
	std::string getLabel(std::size_t i) const;

	/**
	 * @brief Get the vector of values.
	 * @return vector of values.
	 */
	const std::vector<double>& getValues() const;

	/**
	 * @brief Get a value.
	 * @param i [in] the index of the label and value.
	 * @return the value.
	 */
	double getValue(std::size_t i) const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

private:
	std::vector<double> _values;
	std::vector<std::string> _labels;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGVALUES_HPP_ */
