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

#ifndef RWSIM_LOG_LOGCOLLISIONRESULT_HPP_
#define RWSIM_LOG_LOGCOLLISIONRESULT_HPP_

/**
 * @file LogCollisionResult.hpp
 *
 * \copydoc rwsim::log::LogCollisionResult
 */

#include "SimulatorLogEntry.hpp"

#include <rw/proximity/CollisionStrategy.hpp>

namespace rwsim {
namespace log {

class LogPositions;

//! @addtogroup rwsim_log

//! @{
/**
 * @brief Log detailed info from a collision detection.
 */
class LogCollisionResult: public SimulatorLogEntry {
public:
    //! Smart pointer type of LogCollisionResult
    typedef rw::common::Ptr<LogCollisionResult> Ptr;

    //! @copydoc SimulatorLogEntry::SimulatorLogEntry
    LogCollisionResult(SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~LogCollisionResult();

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

	//! @brief The result including info about the ProximityModels.
	struct ResultInfo {
		//! @brief The result of the collision check.
		rw::proximity::CollisionStrategy::Result result;

		//! @brief The name of the first frame.
		std::string frameA;

		//! @brief The name of the second frame.
		std::string frameB;

		//! @brief The geometry ids for the first frame.
		std::vector<std::string> geoNamesA;

		//! @brief The geometry ids for the second frame.
		std::vector<std::string> geoNamesB;

		/**
		 * @brief Compare with other ResultInfo structure.
		 * @param b [in] the other structure.
		 * @return true if equal, false otherwise.
		 */
		bool operator==(const ResultInfo &b) const;
	};

	/**
	 * @brief Retrieve the logged result.
	 * @return the result.
	 */
	const std::vector<ResultInfo>& getResults() const;

	/**
	 * @brief Add the result to this log entry.
	 * @param result [in] the result.
	 */
	void addResult(const rw::proximity::CollisionStrategy::Result& result);

	/**
	 * @brief Add results to this log entry.
	 * @param results [in] list of results.
	 */
	void addResults(const std::vector<rw::proximity::CollisionStrategy::Result>& results);

	/**
	 * @brief Get the positions of the objects.
	 *
	 * This is similar to getLinkedEntries.
	 *
	 * @return the log entry with positions of objects (or NULL if not linked).
	 */
	rw::common::Ptr<LogPositions> getPositions() const;

private:
	rw::common::Ptr<LogPositions> _positions;
	std::vector<ResultInfo> _results;
};
//! @}
} /* namespace log */
} /* namespace rwsim */
#endif /* RWSIM_LOG_LOGCOLLISIONRESULT_HPP_ */
