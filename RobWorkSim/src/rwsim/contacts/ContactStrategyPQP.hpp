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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_

/**
 * @file ContactStrategyPQP.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyPQP
 */

#include "ContactStrategy.hpp"

// Forward declarations
namespace rwlibs { namespace proximitystrategies { class ProximityStrategyPQP; } }

namespace rwsim {
namespace contacts {

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between triangle meshes.
 */
class ContactStrategyPQP: public rwsim::contacts::ContactStrategy {
public:
	//! @brief Strategy used for condensing contacts.
	typedef enum ContactFilter {
		NONE,//!< No filtering.
		MANIFOLD//!< Filtering with manifold.
	} ContactFilter;

	//! @brief Create new strategy.
	ContactStrategyPQP();

	//! @brief Destructor
	virtual ~ContactStrategyPQP();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,ContactStrategyData*,ContactStrategyTracking&) const
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts
	virtual std::vector<Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData& data,
			ContactStrategyTracking& tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::getName
	virtual std::string getName();

	//! @copydoc rwsim::contacts::ContactStrategy::createModel
	virtual rw::proximity::ProximityModel::Ptr createModel();

	//! @copydoc rwsim::contacts::ContactStrategy::destroyModel
	virtual void destroyModel(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false);

	//! @copydoc rwsim::contacts::ContactStrategy::removeGeometry
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

	//! @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::clear
	virtual void clear();

	/**
	 * @brief If strategy should convert all geometry to meshes, or only work on geometry that is already meshes.
	 *
	 * @param matchAll [in] true if strategy should match all geometries (default), false otherwise.
	 */
	virtual void setMatchAll(bool matchAll = true);

	/**
	 * @brief Set the method used to filter contacts (condenses many contacts into fewer contacts).
	 *
	 * @param filter [in] the strategy to use - default is the OBRManifold-strategy.
	 */
	virtual void setContactFilter(ContactFilter filter = MANIFOLD);

	/**
	 * @brief The current distance threshold used by the simulator.
	 *
	 * The default threshold is 0.5 mm.
	 * This can be changed by setting the ContactStrategyPQPThreshold property in the PropertyMap.
	 * If this is not set, the MaxSepDistance property can also be used.
	 *
	 * @return the threshold (positive).
	 */
	virtual double getThreshold() const;

	/**
	 * @brief The current update threshold used for contact tracking.
	 *
	 * The threshold is a absolute threshold that is combined with the motion-relative
	 * thresholds getUpdateThresholdLinear() and getUpdateThresholdAngular().
	 *
	 * Default value of this is 1 mm.
	 *
	 * @return the threshold.
	 */
	virtual double getUpdateThresholdAbsolute() const;

	/**
	 * @brief The current update threshold used for contact tracking.
	 *
	 * The threshold is a factor that is multiplied with the change in relative
	 * displacement of two frames. If the distance between an old and updated contact
	 * is less than this threshold it will be classified as the same contact.
	 *
	 * The default is 2.
	 * The threshold be changed by setting the ContactStrategyPQPUpdateThresholdLinear property in the PropertyMap.
	 *
	 * Note that the getUpdateThresholdAngular is accumulated with this threshold.
	 *
	 * @return the factor.
	 */
	virtual double getUpdateThresholdLinear() const;

	/**
	 * @brief The current update threshold used for contact tracking.
	 *
	 * The threshold is a factor that is multiplied with the angular change of the relative
	 * rotation. If the distance between an old and updated contact is less than this
	 * threshold it will be classified as the same contact.
	 *
	 * The default is 0.25 m/rad.
	 * The threshold can be changed by setting the ContactStrategyPQPUpdateThresholdAngular property in the PropertyMap.
	 *
	 * Note that the getUpdateThresholdLinear is accumulated with this threshold.
	 *
	 * @return the factor.
	 */
	virtual double getUpdateThresholdAngular() const;

private:
	struct Model;
	class TriMeshModel;
	class PQPData;
	class PQPTracking;

	virtual void findContact(std::vector<Contact> &contacts,
				const Model& a,	const rw::math::Transform3D<>& wTa,
				const Model& b,	const rw::math::Transform3D<>& wTb,
				PQPData* data,
				bool distCheck = true) const;

	static std::vector<Contact> manifoldFilter(const std::vector<Contact> &contacts);

	bool _matchAll;
	rwlibs::proximitystrategies::ProximityStrategyPQP* _narrowStrategy;
	ContactFilter _filtering;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYPQP_HPP_ */
