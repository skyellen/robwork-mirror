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

#ifndef RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKING_HPP_
#define RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKING_HPP_

/**
 * @file RWPELogContactTracking.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPELogContactTracking
 */

#include <rwsim/log/SimulatorLogEntry.hpp>

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class RWPELogContactTracking: public rwsim::log::SimulatorLogEntry {
public:
    //! Smart pointer type of RWPELogContactTracking
    typedef rw::common::Ptr<RWPELogContactTracking> Ptr;

    /**
     * @brief Construct new log entry for tracking info.
     * @param parent [in] the log scope.
     */
    RWPELogContactTracking(rwsim::log::SimulatorLogScope* parent);

	//! @brief Destructor.
	virtual ~RWPELogContactTracking();

	//! @copydoc rw::common::Serializable::read
	virtual void read(class rw::common::InputArchive& iarchive, const std::string& id);

	//! @copydoc rw::common::Serializable::write
	virtual void write(class rw::common::OutputArchive& oarchive, const std::string& id) const;

	//! @copydoc SimulatorLogEntry::getType
	virtual std::string getType() const;

	//! @copydoc SimulatorLogEntry::getLinkedEntries
	virtual std::list<SimulatorLogEntry::Ptr> getLinkedEntries() const;

	//! @copydoc SimulatorLogEntry::autoLink
	virtual bool autoLink();

	//! @copydoc SimulatorLogEntry::createNew
	virtual SimulatorLogEntry::Ptr createNew(rwsim::log::SimulatorLogScope* parent) const;

	/**
	 * @brief Get the type id of this entry type.
	 * @return the type id.
	 */
	static std::string getTypeID();

public:
	std::vector<rwsim::contacts::Contact> before;
	std::vector<rwsim::contacts::Contact> after;
	std::vector<rwsim::contacts::Contact> added;
	std::vector<rwsim::contacts::Contact> gone;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKING_HPP_ */
