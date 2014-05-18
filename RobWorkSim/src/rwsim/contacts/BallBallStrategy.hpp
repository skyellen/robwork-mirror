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

#ifndef RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_
#define RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_

/**
 * @file BallBallStrategy.hpp
 *
 * \copydoc rwsim::contacts::BallBallStrategy
 */

#include "ContactStrategy.hpp"

namespace rwsim {
namespace contacts {

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Detection of contacts between balls. Each model can consist of multiple balls.
 */
class BallBallStrategy: public rwsim::contacts::ContactStrategy {
public:
	//! @brief Create new strategy.
	BallBallStrategy();

	//! @brief Destructor
	virtual ~BallBallStrategy();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,ContactStrategyData*,ContactStrategyTracking*) const
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData* data,
			ContactStrategyTracking* tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts
	virtual std::vector<Contact> updateContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData* data,
			ContactStrategyTracking* tracking) const;

	//! @copydoc rwsim::contacts::ContactStrategy::createTracking
	virtual ContactStrategyTracking* createTracking() const;

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

private:
    struct Model;
	class BallModel;
	class BallTracking;

	virtual bool findContact(Contact &c,
			const Model& a,	const rw::math::Transform3D<>& wTa,
			const Model& b,	const rw::math::Transform3D<>& wTb, bool distCheck = true) const;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_ */
