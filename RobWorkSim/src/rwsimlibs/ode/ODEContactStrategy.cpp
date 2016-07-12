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

#include "ODEContactStrategy.hpp"
#include "ODEUtil.hpp"

#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Tube.hpp>

#include <rwsim/contacts/ContactModel.hpp>
#include <rwsim/contacts/ContactStrategyTracking.hpp>

#include <ode/collision.h>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::proximity;
using namespace rwsim::contacts;
using namespace rwsim::simulator;

class ODEContactStrategy::ODEContactModel: public ContactModel {
public:
	typedef rw::common::Ptr<ODEContactModel> Ptr;
	ODEContactModel(ODEContactStrategy* pOwner): ContactModel(pOwner), space(0) { setFrame(NULL); };
	virtual ~ODEContactModel() {};
	virtual std::string getName() const {
		return "ODEContactModel";
	}

public:
	struct ODEModel {
		std::string geoId;
		GeometryData* geo;
		Transform3D<> transform;
		const Frame* frame;
		dGeomID geomId;
		bool movable;
	};

	std::vector<ODEModel> models;
	dSpaceID space;
};

struct ODEContactStrategy::ContactData {
	ODEContactStrategy::ODEContactModel::Ptr mA;
	ODEContactStrategy::ODEContactModel::Ptr mB;
	std::vector<Contact> contacts;
};

struct ODEContactStrategy::TrackInfo {
	TrackInfo(): userData(NULL) {}
	ContactStrategyTracking::UserData::Ptr userData;
	//Vector3D<> posA;
	//Vector3D<> posB;
};

class ODEContactStrategy::ODETracking: public ContactStrategyTracking::StrategyData {
public:
	ODETracking() {};
	virtual ~ODETracking() {};

	virtual const ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const {
		RW_ASSERT(index < info.size());
		return info[index].userData;
	}

	virtual void setUserData(std::size_t index, const ContactStrategyTracking::UserData::Ptr data) {
		RW_ASSERT(index < info.size());
		info[index].userData = data;
	}

	virtual void remove(std::size_t index) {
		RW_ASSERT(index < info.size());
		info.erase(info.begin()+index);
	}

	virtual StrategyData* copy() const {
		ODETracking* tracking = new ODETracking();
		tracking->info = info;
		return tracking;
	}

	virtual std::size_t getSize() const {
		return info.size();
	}

public:
	std::vector<TrackInfo> info;
};

bool ODEContactStrategy::_isODEInitialized = false;

ODEContactStrategy::ODEContactStrategy() {
}

ODEContactStrategy::~ODEContactStrategy() {
}

bool ODEContactStrategy::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::CylinderPrim)
		return true;
	if (geoA->getType() == GeometryData::CylinderPrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	if (geoA->getType() == GeometryData::PlanePrim && geoB->getType() == GeometryData::TubePrim)
		return true;
	if (geoA->getType() == GeometryData::TubePrim && geoB->getType() == GeometryData::PlanePrim)
		return true;
	return false;
}

std::vector<Contact> ODEContactStrategy::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	const ODEContactModel::Ptr mA = a.cast<ODEContactModel>();
	const ODEContactModel::Ptr mB = b.cast<ODEContactModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new ODETracking());
	ODETracking* const odeTracking = dynamic_cast<ODETracking*>(tracking.getStrategyData());
	RW_ASSERT(odeTracking);

	BOOST_FOREACH(const ODEContactModel::ODEModel& model, mA->models) {
		if (model.movable)
			ODEUtil::setODEGeomT3D(model.geomId,wTa*model.transform);
	}
	BOOST_FOREACH(const ODEContactModel::ODEModel& model, mB->models) {
		if (model.movable)
			ODEUtil::setODEGeomT3D(model.geomId,wTb*model.transform);
	}

	ContactData contactData;
	contactData.mA = mA;
	contactData.mB = mB;
	dSpaceCollide2((dGeomID)mA->space, (dGeomID)mB->space, &contactData, &ODEContactStrategy::nearCallback);

	odeTracking->info.resize(contactData.contacts.size());

	std::size_t i = 0;
	BOOST_FOREACH(Contact& contact, contactData.contacts) {
		contact.setTransform(inverse(wTa)*wTb);
		i++;
	}

	return contactData.contacts;
}

std::vector<Contact> ODEContactStrategy::updateContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	if (!tracking.isInitialized())
		tracking.setStrategyData(new ODETracking());
	ODETracking* const odeTracking = dynamic_cast<ODETracking*>(tracking.getStrategyData());
	RW_ASSERT(odeTracking);
	odeTracking->info.clear();
	return std::vector<Contact>();
	//return findContacts(a,wTa,b,wTb,data,tracking);
}

std::string ODEContactStrategy::getName() {
	return "ODEContactStrategy";
}

ProximityModel::Ptr ODEContactStrategy::createModel() {
	const ODEContactModel::Ptr model = ownedPtr(new ODEContactModel(this));
	model->space = dSimpleSpaceCreate(0);
	return model;
}

void ODEContactStrategy::destroyModel(ProximityModel* model) {
	ODEContactModel* bmodel = dynamic_cast<ODEContactModel*>(model);
	RW_ASSERT(bmodel);
	BOOST_FOREACH(const ODEContactModel::ODEModel& model, bmodel->models) {
		dGeomDestroy(model.geomId);
	}
	bmodel->models.clear();
	dSpaceDestroy(bmodel->space);
}

bool ODEContactStrategy::addGeometry(ProximityModel* model, const Geometry& geom) {
	ODEContactModel* bmodel = dynamic_cast<ODEContactModel*>(model);
	RW_ASSERT(bmodel);
	GeometryData::Ptr geomData = geom.getGeometryData();

	ODEContactModel::ODEModel newModel;
	newModel.geoId = geom.getId();
	newModel.transform = geom.getTransform();
	newModel.geo = geomData.get();
	newModel.frame = geom.getFrame();
	if (const Cylinder* const geo = dynamic_cast<Cylinder*>(geomData.get())) {
		newModel.movable = true;
		newModel.geomId = dCreateCylinder(bmodel->space,geo->getRadius(),geo->getHeight());
		bmodel->models.push_back(newModel);
		return true;
	} else if (const Tube* const geo = dynamic_cast<Tube*>(geomData.get())) {
		// Only valid for tube to plane
		newModel.movable = true;
		newModel.geomId = dCreateCylinder(bmodel->space,geo->getInnerRadius(),geo->getHeight());
		bmodel->models.push_back(newModel);
		return true;
	} else if (const Plane* const geo = dynamic_cast<Plane*>(geomData.get())) {
		newModel.movable = false;
		const Vector3D<> n = geo->normal();
		newModel.geomId = dCreatePlane(bmodel->space,n[0],n[1],n[2],geo->d());
		bmodel->models.push_back(newModel);
		return true;
	}
	return false;
}

bool ODEContactStrategy::addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy) {
	return addGeometry(model, *geom);
}

bool ODEContactStrategy::removeGeometry(ProximityModel* model, const std::string& geomId) {
	ODEContactModel* bmodel = dynamic_cast<ODEContactModel*>(model);
	bool removed = false;
	for (std::vector<ODEContactModel::ODEModel>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++) {
		if ((*it).geoId == geomId) {
			dGeomDestroy((*it).geomId);
			bmodel->models.erase(it);
			removed = true;
		}
	}
	return removed;
}

std::vector<std::string> ODEContactStrategy::getGeometryIDs(ProximityModel* model) {
	ODEContactModel* bmodel = dynamic_cast<ODEContactModel*>(model);
	std::vector<std::string> res;
	for (std::vector<ODEContactModel::ODEModel>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++) {
		res.push_back((*it).geoId);
	}
	return res;
}

void ODEContactStrategy::clear() {
	// Nothing to clear
}

void ODEContactStrategy::nearCallback(void *data, dGeomID o1, dGeomID o2) {
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2))
        RW_THROW("ODEContactStrategy: something weird happened when checking collision between spaces!");

	// Missing: we need to check that the geometries belong to different models (multiple geometries in one model must not collide with itself)
	ContactData* const contactData = reinterpret_cast<ContactData*>(data);

	static const int MAX_CONTACTS = 100; // make this dependent of the type of geometries!
    std::vector<dContactGeom> contacts(MAX_CONTACTS);
	const int numc = dCollide(o1, o2, MAX_CONTACTS-1, &contacts[0],	sizeof(dContactGeom));

	if( numc >= MAX_CONTACTS-1 )
		RW_THROW("ODEContactStrategy uses too small collision buffer!");

	const ODEContactModel::ODEModel* model1 = NULL;
	const ODEContactModel::ODEModel* model2 = NULL;
	BOOST_FOREACH(const ODEContactModel::ODEModel& model, contactData->mA->models) {
		if (model1 != NULL)
			break;
		if (model.geomId == o1)
			model1 = &model;
	}
	BOOST_FOREACH(const ODEContactModel::ODEModel& model, contactData->mB->models) {
		if (model2 != NULL)
			break;
		if (model.geomId == o2)
			model2 = &model;
	}
	RW_ASSERT(model1 != NULL && model2 != NULL);

	contactData->contacts.resize(numc);
	RW_ASSERT(numc >= 0);
	for (size_t i = 0; i < (std::size_t)numc; i++) {
		const dContactGeom& con = contacts[i];

		const double normalSize = ODEUtil::toVector3D(con.normal).norm2();
		if(normalSize > 1.0001 || normalSize < 0.9999 ){
			RW_THROW("ODEContactStrategy encountered bad contact normal size!");
		}

		Contact& point = contactData->contacts[i];
		const Vector3D<> n = -normalize( ODEUtil::toVector3D(con.normal) );
		const Vector3D<> p = ODEUtil::toVector3D(con.pos);
		point.setNormal(n);
		if (model1->geo->getType() == GeometryData::PlanePrim || model2->geo->getType() == GeometryData::PlanePrim) {
			if (model1->geo->getType() == GeometryData::TubePrim || model1->geo->getType() == GeometryData::CylinderPrim) {
				point.setPointA(p);
				point.setPointB(p-n*con.depth);
			} else {
				point.setPointB(p);
				point.setPointA(p+n*con.depth);
			}
		} else {
			point.setPointA(p);
			point.setPointB(p-n*con.depth);
		}
		point.setDepth(con.depth);
		point.setModelA(contactData->mA);
		point.setModelB(contactData->mB);
		point.setFrameA(model1->frame);
		point.setFrameB(model2->frame);
	}
}

void ODEContactStrategy::initODE() {
    if(!_isODEInitialized){
        dInitODE2(0);
        _isODEInitialized = true;
    }
}
