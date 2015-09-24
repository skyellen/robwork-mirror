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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGY_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGY_HPP_

/**
 * @file ContactStrategy.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategy
 */

#include "Contact.hpp"
#include "ContactModel.hpp"
#include "ContactStrategyData.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/proximity/ProximityStrategy.hpp>
#include <rw/proximity/ProximityModel.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

namespace rwsim {
namespace contacts {

// Forward declarations
class ContactStrategyTracking;

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief The ContactStrategy is a common interface for different contact strategies.
 *
 * Each contact strategy implements its own contact models, and uses these models to find contacts.
 */
class ContactStrategy: public rw::proximity::ProximityStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactStrategy> Ptr;

	//! @brief Create new contact strategy.
	ContactStrategy() {};

	//! @brief Destruct contact strategy.
	virtual ~ContactStrategy() {};

	/**
	 * @brief Test is this strategy can be used for the given geometries.
	 *
	 * @param geoA [in] geometry data for the first object.
	 * @param geoB [in] geometry data for the second object.
	 * @return true if this strategy can be used for the given geometries.
	 */
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB) = 0;

	/**
	 * @brief Check if there is contact between two contact models.
	 *
	 * @param a [in] model a.
	 * @param wTa [in] transform of model a.
	 * @param b [in] model b.
	 * @param wTb [in] transform of model b.
	 * @return a list of contacts.
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb) const;

	/**
	 * @brief Check if there is contact between two contact models using additional data.
	 *
	 * Use of this function is encouraged if changes between consecutive calls are expected to be small.
	 * This will in some cases allow the detection algorithm to do certain speed-ups.
	 *
	 * @param a [in] model a.
	 * @param wTa [in] transform of model a.
	 * @param b [in] model b.
	 * @param wTb [in] transform of model b.
	 * @param data [in/out] allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @return a list of contacts.
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data) const;

	/**
	 * @brief Check if there is contact between two contact models using additional data and tracking of contacts.
	 *
	 * Use of this function is encouraged if changes between consecutive calls are expected to be small.
	 * This will in some cases allow the detection algorithm to do certain speed-ups.
	 *
	 * @param a [in] model a.
	 * @param wTa [in] transform of model a.
	 * @param b [in] model b.
	 * @param wTb [in] transform of model b.
	 * @param data [in/out] allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @param tracking [in/out] meta-data for previously found contacts.
	 * @return a list of contacts.
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking) const = 0;

	/**
	 * @brief Update known contacts between two contact models.
	 *
	 * @param a [in] model a.
	 * @param wTa [in] current transform of model a.
	 * @param b [in] model b.
	 * @param wTb [in] current transform of model b.
	 * @param data [in/out] allows caching between contact detection calls,
	 * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
	 * @param tracking [in/out] meta-data for previously found contacts.
	 * @return a list of updated contacts.
	 */
	virtual std::vector<Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking) const = 0;

	/**
	 * @brief Get the name of the strategy as a string.
	 *
	 * @return name of the strategy.
	 */
	virtual std::string getName() = 0;

	//! @copydoc rw::proximity::ProximityStrategy::createModel
	virtual rw::proximity::ProximityModel::Ptr createModel() = 0;

	//! @copydoc rw::proximity::ProximityStrategy::destroyModel
	virtual void destroyModel(rw::proximity::ProximityModel* model) = 0;

	//! @copydoc rw::proximity::ProximityStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom) = 0;

	//! @copydoc rw::proximity::ProximityStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false) = 0;

	//! @copydoc rw::proximity::ProximityStrategy::removeGeometry
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId) = 0;

	//! @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model) = 0;

	//! @copydoc rw::proximity::ProximityStrategy::clear
	virtual void clear() = 0;

	/**
	 * @brief Get the properties used by the contact strategy.
	 *
	 * @return reference to the property map.
	 */
	virtual rw::common::PropertyMap& getPropertyMap();

	/**
	 * @brief Get the properties used by the contact strategy.
	 *
	 * @return the property map.
	 */
	virtual const rw::common::PropertyMap& getPropertyMap() const;

	/**
	 * @brief Set which properties the contact strategy should use.
	 *
	 * @param map [in] the property map to get properties from.
	 */
	virtual void setPropertyMap(const rw::common::PropertyMap& map);

protected:
	rw::common::PropertyMap _propertyMap;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGY_HPP_ */
