/*
 * ContactStrategy.hpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#ifndef CONTACTSTRATEGY_HPP_
#define CONTACTSTRATEGY_HPP_

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

class ContactStrategy: public rw::proximity::ProximityStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactStrategy> Ptr;

	ContactStrategy() {};
	virtual ~ContactStrategy() {};

	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB) = 0;

	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb) = 0;

	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData &data) = 0;

	virtual std::string getName() = 0;

	virtual rw::proximity::ProximityModel::Ptr createModel() = 0;
	virtual void destroyModel(rw::proximity::ProximityModel* model) = 0;
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom) = 0;
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false) = 0;
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId) = 0;
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model) = 0;
	virtual void clear() = 0;
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTSTRATEGY_HPP_ */
