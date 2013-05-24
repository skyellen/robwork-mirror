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
	 * A default broad-phase collision detector is used (a static filter list).
	 *
	 * @note The strategy rule table will be empty and no contacts will be found before strategies are added.
	 *
	 * @param workcell [in] the workcell.
	 */
	ContactDetector(rw::models::WorkCell::Ptr workcell);

	/**
	 * @brief Contact detector for a workcell with specific broad-phase filter.
	 *
	 * @note The strategy rule table will be empty and no contacts will be found before strategies are added.
	 *
	 * @param workcell [in] the workcell.
	 * @param filter [in] broad-phase filter to remove frames that are obviously not colliding.
	 */
	ContactDetector(rw::models::WorkCell::Ptr workcell,
			rw::proximity::ProximityFilterStrategy::Ptr filter);

	/**
	 * @brief Destruct contact detector.
	 *
	 * The strategy table and stored contact models is cleared.
	 */
	virtual ~ContactDetector();

	/**
	 * @brief Find contacts in workcell.
	 *
	 * @param state [in] The state for which to check for contacts.
	 * @return a vector of contacts, some might be subclasses of the Contact class.
	 */
	std::vector<Contact> findContacts(const rw::kinematics::State& state) const;

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
	std::vector<Contact> findContacts(const rw::kinematics::State& state,
			ContactDetectorData &data) const;

	/**
	 * @brief The broad-phase filter strategy used by the contact detector.
	 */
	rw::proximity::ProximityFilterStrategy::Ptr getProximityFilterStrategy() const;

	/**
	 * @name Strategy table functions.
	 * Functions used to construct and edit the strategy table.
	 */
	///@{
	/**
	 * @brief Get the complete contact strategy table.
	 *
	 * @return The strategy table used.
	 */
	StrategyTable getContactStategies() const;

	/**
	 * @brief Add a strategy to the strategy table that matches all frames.
	 *
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	void addContactStrategy(ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy that is only used for frames matching one rule.
	 *
	 * @param rule [in] The rule for the frame names that this strategy applies to.
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	void addContactStrategy(rw::proximity::ProximitySetupRule rule,
			ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy that is used for frames that match a set of rules.
	 *
	 * @param rule [in] The rules for the frame names that this strategy applies to.
	 * @param strategy [in/out] The strategy to add. Relevant ContactModels are automatically created.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	void addContactStrategy(rw::proximity::ProximitySetup rules,
			ContactStrategy::Ptr strategy, std::size_t priority = 0);

	/**
	 * @brief Add a strategy from an existing strategy table rule.
	 *
	 * @param strategy [in/out] The strategy to add. The priority will be changed automatically,
	 * and relevant ContactModels are created automatically if not already present.
	 * @param priority [in] the priority of the new strategy (default is maximum priority - 0).
	 */
	void addContactStrategy(StrategyTableRow &strategy, std::size_t priority = 0);

	/**
	 * @brief Remove the strategy with a certain priority in the table.
	 *
	 * Contact models will removed for the relevant strategy, and updated for the other strategies in the table.
	 *
	 * @param priority the priority to remove.
	 */
	void removeContactStrategy(std::size_t priority = 0);

	/**
	 * @brief Remove all strategies and contact models in the table.
	 */
	void clearStrategies();

	/**
	 * @brief Set a complete strategy table.
	 *
	 * @param strategies [in] the strategy table.
	 */
	void setContactStrategies(StrategyTable strategies);

	/**
	 * @brief Auto-generate a suitable general-purpose strategy table.
	 *
	 * This function will generate the most suitable default contact strategy for the workcell.
	 */
	void setDefaultStrategies();

	/**
	 * @brief Print the current strategy table to standard output.
	 */
	void printStrategyTable() const;
	///@}

private:
	struct Cell {
		enum ALIGNMENT {
			LEFT, RIGHT
		};
		Cell() :
				alignment(LEFT) {
		}
		Cell(std::string string) :
				alignment(LEFT) {
			strings.push_back(string);
		}
		std::vector<std::string> strings;
		ALIGNMENT alignment;
	};
	static void printTable(const std::vector<std::vector<Cell> > &table,
			bool header = false);
	void initializeGeometryMap();
	void initializeModels(StrategyTableRow &strategy);

	rw::models::WorkCell::Ptr _wc;
	rw::proximity::ProximityFilterStrategy::Ptr _bpfilter;
	StrategyTable _strategies;

	rw::kinematics::FrameMap<std::vector<rw::geometry::Geometry::Ptr> > _frameToGeo;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTDETECTOR_HPP_ */