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

#include "ContactModel.hpp"
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
	/**
	 * @brief Create new strategy.
	 */
	BallBallStrategy();

	/**
	 * @brief Destructor
	 */
	virtual ~BallBallStrategy();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::match
	 */
	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,rw::proximity::ProximityModel*,const rw::math::Transform3D<>&)
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,rw::proximity::ProximityModel*,const rw::math::Transform3D<>&,ContactStrategyData&)
	 */
	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData &data);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::getName
	 */
	virtual std::string getName();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::createModel
	 */
	virtual rw::proximity::ProximityModel::Ptr createModel();

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::destroyModel
	 */
    virtual void destroyModel(rw::proximity::ProximityModel* model);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	 */
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	 */
    virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::removeGeometry
	 */
    virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
	 */
    virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::clear
	 */
    virtual void clear();

    /**
     * @brief For modelling of a single ball.
     */
	struct Model {
    	//! The geometry id of the ball.
		std::string geoId;

		//! Radius of the ball.
		double radius;

		//! The center of the ball.
		rw::math::Vector3D<> center;

		//! The frame
		const rw::kinematics::Frame* frame;
	};

	/**
	 * @brief The contact model used by this strategy. One model can consist of multiple balls if desired.
	 */
	class BallModel: public ContactModel {
	public:
		/**
		 * @brief Construct new model for object consisting of one or more balls.
		 *
		 * @param owner [in] the strategy that owns this model.
		 */
		BallModel(ContactStrategy *owner):
			ContactModel(owner)
		{
		}

		/**
		 * @copydoc rwsim::contacts::ContactModel::getName
		 */
		virtual std::string getName() const { return "BallModel"; };

		/**
		 * @brief List of ball models belonging to this model.
		 */
		std::vector<Model> models;
	};
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_BALLBALLSTRATEGY_HPP_ */
