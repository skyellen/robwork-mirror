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

#ifndef RWSIM_CONTACTS_CONTACTSTRATEGYGEOMETRY_HPP_
#define RWSIM_CONTACTS_CONTACTSTRATEGYGEOMETRY_HPP_

/**
 * @file ContactStrategyGeometry.hpp
 *
 * \copydoc rwsim::contacts::ContactStrategyGeometry
 */

#include "ContactStrategy.hpp"
#include "ContactModelGeometry.hpp"

namespace rwsim {
namespace contacts {
//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Generic contact strategy that find contacts between two different types of geometry.
 */
template <class A, class B>
class ContactStrategyGeometry: public rwsim::contacts::ContactStrategy {
protected:
	typedef ContactModelGeometry<A, B> GeometryModel;

public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactStrategyGeometry<A, B> > Ptr;

	/**
	 * @brief Construct new strategy
	 */
	ContactStrategyGeometry() {}

	/**
	 * @brief Destructor
	 */
	virtual ~ContactStrategyGeometry() {};

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::createModel
	 */
	virtual rw::proximity::ProximityModel::Ptr createModel() {
		return rw::common::ownedPtr(new GeometryModel(this));
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::destroyModel
	 */
	virtual void destroyModel(rw::proximity::ProximityModel* model) {
		GeometryModel* bmodel = dynamic_cast<GeometryModel*>(model);
		RW_ASSERT(bmodel);
		bmodel->modelsA.clear();
		bmodel->modelsB.clear();
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,const rw::geometry::Geometry&)
	 */
	virtual bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom) {
		GeometryModel* bmodel = dynamic_cast<GeometryModel*>(model);
		RW_ASSERT(bmodel);
		rw::geometry::GeometryData::Ptr geomData = geom.getGeometryData();
		typename GeometryModel::TypeA newModelA;
		typename GeometryModel::TypeB newModelB;
		if (A geo = dynamic_cast<A>(geomData.get())) {
			newModelA.geoId = geom.getId();
			newModelA.transform = geom.getTransform();
			newModelA.geo = geo;
			newModelA.frame = geom.getFrame();
			bmodel->modelsA.push_back(newModelA);
			return true;
		}
		if (B geo = dynamic_cast<B>(geomData.get())) {
			newModelB.geoId = geom.getId();
			newModelB.transform = geom.getTransform();
			newModelB.geo = geo;
			newModelB.frame = geom.getFrame();
			bmodel->modelsB.push_back(newModelB);
			return true;
		}
		return false;
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::addGeometry(rw::proximity::ProximityModel*,rw::geometry::Geometry::Ptr,bool)
	 */
	virtual bool addGeometry(rw::proximity::ProximityModel* model, rw::geometry::Geometry::Ptr geom, bool forceCopy=false) {
		return addGeometry(model, *geom);
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::removeGeometry
	 */
	virtual bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId) {
		GeometryModel* bmodel = (GeometryModel*) model;
		bool removed = false;
		for (typename std::vector<typename GeometryModel::TypeA>::iterator it = bmodel->modelsA.begin(); it < bmodel->modelsA.end(); it++) {
			if ((*it).geoId == geomId) {
				bmodel->modelsA.erase(it);
				removed = true;
			}
		}
		for (typename std::vector<typename GeometryModel::TypeB>::iterator it = bmodel->modelsB.begin(); it < bmodel->modelsB.end(); it++) {
			if ((*it).geoId == geomId) {
				bmodel->modelsB.erase(it);
				removed = true;
			}
		}
		return removed;
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::getGeometryIDs
	 */
	virtual std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model) {
		GeometryModel* bmodel = (GeometryModel*) model;
		std::vector<std::string> res;
		for (typename std::vector<typename GeometryModel::TypeA>::iterator it = bmodel->modelsA.begin(); it < bmodel->modelsA.end(); it++)
			res.push_back((*it).geoId);
		for (typename std::vector<typename GeometryModel::TypeB>::iterator it = bmodel->modelsB.begin(); it < bmodel->modelsB.end(); it++)
			res.push_back((*it).geoId);
		return res;
	}

	/**
	 * @copydoc rwsim::contacts::ContactStrategy::clear
	 */
	virtual void clear() {
		// Nothing to clear
	}
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_CONTACTSTRATEGYGEOMETRY_HPP_ */
