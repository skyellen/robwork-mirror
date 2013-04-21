/*
 * ContactDetector.hpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#ifndef CONTACTDETECTOR_HPP_
#define CONTACTDETECTOR_HPP_

#include "ContactDetectorData.hpp"
#include "ContactStrategy.hpp"
#include "ContactModel.hpp"
#include "Contact.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/ProximitySetupRule.hpp>
#include <rw/proximity/ProximitySetup.hpp>
#include <rw/proximity/ProximityFilterStrategy.hpp>

namespace rwsim {
namespace contacts {

class ContactDetector {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactDetector> Ptr;

    struct StrategyTable {
		std::size_t priority;
    	rw::proximity::ProximitySetup rules;
    	ContactStrategy::Ptr strategy;
    	rw::kinematics::FrameMap<std::map<std::string, ContactModel::Ptr> > models;
    };

	ContactDetector(rw::models::WorkCell::Ptr workcell);
	ContactDetector(rw::models::WorkCell::Ptr workcell, rw::proximity::ProximityFilterStrategy::Ptr filter);
	virtual ~ContactDetector();

	rw::proximity::ProximityFilterStrategy::Ptr getProximityFilterStrategy() const;

	std::list<StrategyTable> getContactStategies() const;
	void addContactStrategy(ContactStrategy::Ptr strategy, std::size_t pri = 0);
	void addContactStrategy(rw::proximity::ProximitySetupRule rule, ContactStrategy::Ptr strategy, std::size_t pri = 0);
	void addContactStrategy(rw::proximity::ProximitySetup rules, ContactStrategy::Ptr strategy, std::size_t pri = 0);
	void addContactStrategy(StrategyTable strategy, std::size_t pri = 0);
	void removeContactStrategy(std::size_t pri = 0);
	void clearStrategies();
	void setContactStrategies(std::list<StrategyTable> strategies);
	void setDefaultStrategies();

    std::vector<Contact> findContacts(const rw::kinematics::State& state) const;
    std::vector<Contact> findContacts(const rw::kinematics::State& state, ContactDetectorData &data) const;

private:
    void initializeMap();

	rw::models::WorkCell::Ptr _wc;
	rw::proximity::ProximityFilterStrategy::Ptr _bpfilter;
	std::list<StrategyTable> _strategies;

	rw::kinematics::FrameMap<std::vector<rw::geometry::Geometry::Ptr> > _frameToGeo;
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTDETECTOR_HPP_ */
