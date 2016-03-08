/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_ODE_ODECONTACTSTRATEGY_HPP_
#define RWSIMLIBS_ODE_ODECONTACTSTRATEGY_HPP_

/**
 * @file ODEContactStrategy.hpp
 *
 * \copydoc rwsim::simulator::ODEContactStrategy
 */

#include <rwsim/contacts/ContactStrategy.hpp>

#include <ode/collision.h>

namespace rwsim {
namespace simulator {
//! @addtogroup rwsimlibs_ode

//! @{
/**
 * @brief Detection of contacts using the internal ODE collision detector.
 *
 * Please note that currently only Tube to Plane and Cylinder to Plane detection is supported.
 *
 * Also remember to call the static initODE function before doing any detection.
 * This should only be done once, and if the ODESimulator is used, ODE will be initialized there.
 * If using the ODEContactStrategy with other simulators, always remember to call initODE.
 */
class ODEContactStrategy: public rwsim::contacts::ContactStrategy {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ODEContactStrategy> Ptr;

	//! @brief Create new strategy.
	ODEContactStrategy();

	//! @brief Destructor
	virtual ~ODEContactStrategy();

	//! @copydoc rwsim::contacts::ContactStrategy::match
	virtual bool match(rw::common::Ptr<const rw::geometry::GeometryData> geoA, rw::common::Ptr<const rw::geometry::GeometryData> geoB);

	//! @copydoc rwsim::contacts::ContactStrategy::findContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rwsim::contacts::ContactStrategyData& data,rwsim::contacts::ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope*)
	virtual std::vector<rwsim::contacts::Contact> findContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

	//! @copydoc rwsim::contacts::ContactStrategy::updateContacts(rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rw::proximity::ProximityModel::Ptr,const rw::math::Transform3D<>&,rwsim::contacts::ContactStrategyData& data,rwsim::contacts::ContactStrategyTracking& tracking,rwsim::log::SimulatorLogScope*)
	virtual std::vector<rwsim::contacts::Contact> updateContacts(
			rw::proximity::ProximityModel::Ptr a,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
			const rw::math::Transform3D<>& wTb,
			rwsim::contacts::ContactStrategyData& data,
			rwsim::contacts::ContactStrategyTracking& tracking,
			rwsim::log::SimulatorLogScope* log = NULL) const;

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

	//! @brief If ODE is not already initialized please call this function before doing any contact detection.
	static void initODE();

private:
	class ODEContactModel;
	struct ContactData;
	struct TrackInfo;
	class ODETracking;

	static void nearCallback(void *data, dGeomID o1, dGeomID o2);

	static bool _isODEInitialized;
};
//! @}
} /* namespace simulator */
} /* namespace rwsim */
#endif /* RWSIMLIBS_ODE_ODECONTACTSTRATEGY_HPP_ */
