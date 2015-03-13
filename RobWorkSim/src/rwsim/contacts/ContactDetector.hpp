/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_CONTACTS_CONTACTDETECTOR_HPP_
#define RWSIM_CONTACTS_CONTACTDETECTOR_HPP_

/**
 * @file ContactDetector.hpp
 *
 * \copydoc rwsim::contacts::ContactDetector
 */

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

// Forward declarations
class ContactDetectorTracking;

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief The ContactDetector allows detailed control of the strategies used for contact detection between specific frames and geometry types.
 *
 * A concrete strategy for contact detection is chosen by a rule-based lookup in a strategy table.
 * For general purpose usage the contact detector can auto-generate the most suitable set of rules for a given workcell.
 * The user can alter this table to specific needs, or create its own strategy table from scratch.
 *
 * Each rule has a priority from 0 and upwards, where 0 is the highest priority. When finding contacts, the rule with priority 0 will be
 * tested first. If no match is found, the next rule in the table will be tested.
 *
 * The first condition for a match, will be that the frame names match a specific pattern. If the frame names match, each combination of
 * geometries for the two frames is tested by calling the boolean match function on the ContactStrategy. If the geometries can not be handled
 * by the strategy, the next strategy in the table will be tested. If the geometries can be handled by the strategy, this strategy is used for
 * contact detection between this specific combination of geometries.
 *
 * This way the user can easily extend the default contact detections strategy with own strategies. These strategies can be applied for specific
 * frame names, specific geometry types or both.
 */
class ContactDetector {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactDetector> Ptr;

	/**
	 * @brief One row in the strategy table, with a priority, match-rules, a strategy and associated ContactModels.
	 */
	struct StrategyTableRow {
		//! The priority of this rule in the strategy table, where 0 is the highest priority.
		std::size_t priority;

		//! The match rules for the frame names that this strategy rule applies to.
		rw::proximity::ProximitySetup rules;

		//! The contact strategy to use when the rules match.
		ContactStrategy::Ptr strategy;

		//! A map of ContactModels for use with this strategy. ContactModels are specific for the used strategy.
		rw::kinematics::FrameMap<std::map<std::string, ContactModel::Ptr> > models;
	};

	//! @brief Type for the strategy table.
	typedef std::list<StrategyTableRow> StrategyTable;

	/**
	 * @brief Contact detector for a workcell.
	 *
	 * If no broad-phase filter is given, a default will be created for the workcell.
	 *
	 * @note The strategy rule table will be empty and no contacts will be found before strategies are added.
	 *
	 * @param workcell [in] the workcell.
	 * @param filter [in] broad-phase filter to remove frames that are obviously not colliding.
	 */
	ContactDetector(rw::models::WorkCell::Ptr workcell,
			rw::proximity::ProximityFilterStrategy::Ptr filter = NULL);

	/**
	 * @brief Destruct contact detector.
	 *
	 * The strategy table and stored contact models is cleared.
	 */
	virtual ~ContactDetector();

	/**
	 * @brief Set a new broad-phase filter.
	 * @param filter [in] broad-phase filter to remove frames that are obviously not colliding.
	 */
	void setProximityFilterStrategy(rw::proximity::ProximityFilterStrategy::Ptr filter);

	/**
	 * @brief Find contacts in workcell.
	 *
	 * @param state [in] The state for which to check for contacts.
	 * @return a vector of contacts, some might be subclasses of the Contact class.
	 */
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state);

	/**
	 * @brief Find contacts in workcell.
	 *
	 * Use of this function is encouraged if changes between consecutive calls are expected to be small.
	 * This will allow the detection algorithms to do certain speed-ups.
	 *
	 * @param state [in] The state for which to check for contacts.
	 * @param data [in/out] Allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @return a vector of contacts, some might be subclasses of the Contact class.
	 */
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state,
			ContactDetectorData &data);

	/**
	 * @brief Find contacts in workcell while tracking known contacts.
	 *
	 * @param state [in] the state to find contacts for.
	 * @param data [in/out] allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @param tracking [in/out] the tracking data with information about known contacts.
	 * @return a vector of new contacts.
	 */
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state,
			ContactDetectorData &data, ContactDetectorTracking& tracking);

	/**
	 * @brief Updates previously found contacts.
	 *
	 * @param state [in] the new state to find the updated contacts for.
	 * @param data [in/out] allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @param tracking [in/out] the tracking data with information about known contacts.
	 * @return a vector of contacts.
	 */
	virtual std::vector<Contact> updateContacts(const rw::kinematics::State& state,
			ContactDetectorData &data, ContactDetectorTracking& tracking);

	/**
	 * @brief The broad-phase filter strategy used by the contact detector.
	 */
	virtual rw::proximity::ProximityFilterStrategy::Ptr getProximityFilterStrategy() const;

	/**
	 * @brief The number of seconds measured used in contact detection.
	 * @return the value of the timer in seconds.
	 */
	virtual double getTimer() const;

	/**
	 * @brief Set the value of a timer that will measure time used during contact detection.
	 * @param value [in] the value to set the time to (seconds)
	 */
	virtual void setTimer(double value = 0);

	/**
	 * @name Strategy table functions.
	 * Functions used to construct and edit the strategy table.
	 */
	///@{
	/**
	 * @brief Get the complete contact strategy table.
	 * @return The strategy table used.
	 */
	virtual StrategyTable getContactStategies() const;

	/**
	 * @brief Get the contact strategies that match the given frame names.
	 * @param frameA [in] the name of the first frame.
	 * @param frameB [in] the name of the second frame.
	 * @return a new strategy table that includes only strategies matching the given frame pair.
	 */

	virtual StrategyTable getContactStrategies(const std::string& frameA, const std::string& frameB) const;

	/**
	 * @brief Get the contact strategies that match the given frame names and geometries.
	 * @param frameA [in] the name of the first frame.
	 * @param geometryA [in] the first geometry.
	 * @param frameB [in] the name of the second frame.
	 * @param geometryB [in] the second geometry.
	 * @return a new strategy table that includes only strategies matching the given frame pair, and uses the given geometries.
	 */
	virtual StrategyTable getContactStrategies(const std::string& frameA, rw::common::Ptr<const rw::geometry::Geometry> geometryA, const std::string& frameB, rw::common::Ptr<const rw::geometry::Geometry> geometryB) const;

	/**
	 * @brief Add a strategy to the strategy table that matches all frames.
	 *
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	virtual void addContactStrategy(ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy that is only used for frames matching one rule.
	 *
	 * @param rule [in] The rule for the frame names that this strategy applies to.
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	virtual void addContactStrategy(rw::proximity::ProximitySetupRule rule,
			ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy that is used for frames that match a set of rules.
	 *
	 * @param rule [in] The rules for the frame names that this strategy applies to.
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	virtual void addContactStrategy(rw::proximity::ProximitySetup rules,
			ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy from an existing strategy table rule.
	 *
	 * @param strategy [in/out] The strategy to add. The priority will be changed automatically,
	 * and relevant ContactModels are created automatically if not already present.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	virtual void addContactStrategy(StrategyTableRow &strategy, std::size_t priority = 0);

	/**
	 * @brief Remove the strategy with a certain priority in the table.
	 *
	 * Contact models will removed for the relevant strategy, and updated for the other strategies in the table.
	 *
	 * @param priority the priority to remove.
	 */
	virtual void removeContactStrategy(std::size_t priority = 0);

	/**
	 * @brief Remove all strategies and contact models in the table.
	 */
	virtual void clearStrategies();

	/**
	 * @brief Set a complete strategy table.
	 *
	 * @param strategies [in] the strategy table.
	 */
	virtual void setContactStrategies(StrategyTable strategies);

	/**
	 * @brief Auto-generate a suitable general-purpose strategy table.
	 *
	 * This function will generate the most suitable default contact strategy for the workcell.
	 */
	virtual void setDefaultStrategies();

	/**
	 * @brief Auto-generate a suitable general-purpose strategy table with strategies using the given properties.
	 *
	 * This function will generate the most suitable default contact strategy for the workcell.
	 *
	 * @param map [in] the PropertyMap to use by all the strategies.
	 */
	virtual void setDefaultStrategies(const rw::common::PropertyMap& map);

	/**
	 * @brief Print the current strategy table to standard output.
	 */
	virtual void printStrategyTable() const;

	/**
	 * @brief Print the current strategy table to given output stream.
	 */
	virtual void printStrategyTable(std::ostream& out) const;
	///@}

	/**
	 * @brief Create a default workcell from a workcell, where the default strategies has been set.
	 * @param workcell [in] the workcell to create detector for.
	 * @return a new contact detector.
	 */
	static ContactDetector::Ptr makeDefault(rw::models::WorkCell::Ptr workcell);

	/**
	 * @brief Create a default workcell from a workcell, where the default strategies has been set.
	 * @param workcell [in] the workcell to create detector for.
	 * @param map [in] the map to take properties from.
	 * @return a new contact detector.
	 */
	static ContactDetector::Ptr makeDefault(rw::models::WorkCell::Ptr workcell, const rw::common::PropertyMap& map);

	/**
	 * @brief Stream operator.
	 * @param out [in/out] the stream to write to.
	 * @param detector [in] the detector to print strategy table for.
	 * @return the same ostream as out parameter.
	 */
	friend std::ostream& operator<<(std::ostream& out, const ContactDetector& detector) {
		detector.printStrategyTable(out);
		return out;
	}

	/**
	 * @brief Stream operator.
	 * @param out [in/out] the stream to write to.
	 * @param detector [in] the detector to print strategy table for.
	 * @return the same ostream as out parameter.
	 */
	friend std::ostream& operator<<(std::ostream& out, ContactDetector::Ptr detector) {
		detector->printStrategyTable(out);
		return out;
	}

private:
	struct Cell;
	void constructTable(std::vector<std::vector<Cell> >& table) const;
	static void printTable(const std::vector<std::vector<Cell> > &table,
			std::ostream& out, bool header = false);
	void initializeGeometryMap();
	void initializeModels(StrategyTableRow &strategy);

	rw::models::WorkCell::Ptr _wc;
	rw::proximity::ProximityFilterStrategy::Ptr _bpfilter;
	StrategyTable _strategies;

	rw::kinematics::FrameMap<std::vector<rw::geometry::Geometry::Ptr> > _frameToGeo;

	double _timer;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTDETECTOR_HPP_ */
