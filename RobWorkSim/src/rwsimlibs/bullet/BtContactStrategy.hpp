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

#ifndef RWSIMLIBS_BULLET_BTCONTACTSTRATEGY_HPP_
#define RWSIMLIBS_BULLET_BTCONTACTSTRATEGY_HPP_

/**
 * @file BtContactStrategy.hpp
 *
 * \copydoc rwsimlibs::bullet::BtContactStrategy
 */

#include <rwsim/contacts/ContactStrategy.hpp>

class btCollisionDispatcher;

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief Detection of contacts using the internal Bullet collision detector.
 */
class BtContactStrategy: public rwsim::contacts::ContactStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<BtContactStrategy> Ptr;

	//! @brief Create new strategy.
	BtContactStrategy();

	//! @brief Destructor
	virtual ~BtContactStrategy();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rwsim::contacts::ContactStrategyData& data,rwsim::contacts::ContactStrategyTracking& tracking)
	virtual std::vector<rwsim::contacts::Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rwsim::contacts::ContactStrategyData& data,rwsim::contacts::ContactStrategyTracking& tracking)
	virtual std::vector<rwsim::contacts::Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::getName
	virtual std::string getName();

	//! @copydoc rwsim::contacts::ContactStrategy::createModel
	virtual rw::proximity::ProximityModel::Ptr createModel();

	//! @copydoc rwsim::contacts::ContactStrategy::destroyModel
	virtual void destroyModel(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

	//! @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=true);

	//! @copydoc rwsim::contacts::ContactStrategy::removeGeometry
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

	//! @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

	//! @copydoc rwsim::contacts::ContactStrategy::clear
	virtual void clear();

	/**
	 * @brief The current distance threshold used by the simulator.
	 *
	 * This is the collision margin set for all btCollisionShapes in Bullet.
	 * This can be changed by setting the BtContactStrategyThreshold property in the PropertyMap.
	 * If this is not set, the MaxSepDistance property can also be used.
	 *
	 * @return the threshold (positive).
	 */
	virtual double getThreshold() const;

private:
	btCollisionDispatcher* const _dispatcher;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTCONTACTSTRATEGY_HPP_ */
