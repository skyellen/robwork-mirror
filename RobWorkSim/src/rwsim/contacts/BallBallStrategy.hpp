/*
 * BallBallStrategy.hpp
 *
 *  Created on: 19/04/2013
 *      Author: thomas
 */

#ifndef BALLBALLSTRATEGY_HPP_
#define BALLBALLSTRATEGY_HPP_

#include "ContactModel.hpp"
#include "ContactStrategy.hpp"

namespace rwsim {
namespace contacts {

class BallBallStrategy: public rwsim::contacts::ContactStrategy {
public:
	BallBallStrategy();
	virtual ~BallBallStrategy();

	virtual bool match(rw::geometry::GeometryData::Ptr geoA, rw::geometry::GeometryData::Ptr geoB);

	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb);

	virtual std::vector<Contact> findContacts(
			rw::proximity::ProximityModel* a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel* b,
			const rw::math::Transform3D<>& wTb,
			ContactStrategyData &data);

	virtual std::string getName();

	virtual rw::proximity::ProximityModel::Ptr createModel();
    virtual void destroyModel(rw::proximity::ProximityModel* model);
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);
    virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false);
    virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);
    virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);
    virtual void clear();

private:
	struct Model {
		std::string geoId;
		double radius;
		rw::math::Vector3D<> center;
	};

	class BallModel : public ContactModel {
	public:
		BallModel(ProximityStrategy *owner):
			ContactModel(owner)
		{
		}

		virtual std::string getName() { return "BallModel";	};

		std::vector<Model> models;
	};
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* BALLBALLSTRATEGY_HPP_ */
