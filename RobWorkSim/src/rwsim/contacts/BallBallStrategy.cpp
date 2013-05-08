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

#include "BallBallStrategy.hpp"

#include <rw/geometry/Sphere.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwsim::contacts;

BallBallStrategy::BallBallStrategy()
{
}

BallBallStrategy::~BallBallStrategy()
{
}

bool BallBallStrategy::match(GeometryData::Ptr geoA, GeometryData::Ptr geoB) {
	if (geoA->getType() == GeometryData::SpherePrim && geoB->getType() == GeometryData::SpherePrim)
		return true;
	return false;
}

std::vector<Contact> BallBallStrategy::findContacts(ProximityModel* a, const Transform3D<>& wTa, ProximityModel* b, const Transform3D<>& wTb)
{
	ContactStrategyData data;
	return findContacts(a,wTa,b,wTb,data);
}

std::vector<Contact> BallBallStrategy::findContacts(ProximityModel* a, const Transform3D<>& wTa, ProximityModel* b, const Transform3D<>& wTb, ContactStrategyData &data)
{
	std::vector<Contact> res;
	BallModel* mA = (BallModel*) a;
	BallModel* mB = (BallModel*) b;
	for (std::vector<Model>::iterator itA = mA->models.begin(); itA < mA->models.end(); itA++) {
		for (std::vector<Model>::iterator itB = mB->models.begin(); itB < mB->models.end(); itB++) {
			Model modelA = *itA;
			Model modelB = *itB;
			Vector3D<> cA = wTa.R()*modelA.center+wTa.P();
			Vector3D<> cB = wTb.R()*modelB.center+wTb.P();
			Vector3D<> cVec = cB-cA;
			double cVecLen = cVec.norm2();
			if (cVecLen <= modelA.radius + modelB.radius) {
				Vector3D<> cVecNorm = cVec/cVecLen;
				Contact c;
				c.setModelA(mA);
				c.setModelB(mB);
				c.setPointA(cA+modelA.radius*cVecNorm);
				c.setPointB(cB-modelB.radius*cVecNorm);
				c.setNormal(cVecNorm);
				c.setDepth(modelA.radius + modelB.radius - cVecLen);
				c.setTransform(inverse(wTa)*wTb);
				res.push_back(c);
			}
		}
	}
	return res;
}

std::string BallBallStrategy::getName() {
	return "BallBallStrategy";
}

ProximityModel::Ptr BallBallStrategy::createModel() {
	return ownedPtr(new BallModel(this));
}

void BallBallStrategy::destroyModel(ProximityModel* model) {
	BallModel* bmodel = (BallModel*) model;
	bmodel->models.clear();
}

bool BallBallStrategy::addGeometry(ProximityModel* model, const Geometry& geom) {
	BallModel* bmodel = (BallModel*) model;
	GeometryData::Ptr geomData = geom.getGeometryData();
	if (geomData->getType() != GeometryData::SpherePrim)
		return false;
	Sphere* sData = (Sphere*) geomData.get();
	Model newModel;
	newModel.geoId = geom.getId();
	newModel.center = geom.getTransform().P();
	newModel.radius = sData->getRadius()*geom.getScale();
	bmodel->models.push_back(newModel);
	return true;
}

bool BallBallStrategy::addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy) {
	return addGeometry(model, *geom);
}

bool BallBallStrategy::removeGeometry(ProximityModel* model, const std::string& geomId) {
	BallModel* bmodel = (BallModel*) model;
	for (std::vector<Model>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++) {
		if ((*it).geoId == geomId) {
			bmodel->models.erase(it);
			return true;
		}
	}
	return false;
}

std::vector<std::string> BallBallStrategy::getGeometryIDs(ProximityModel* model) {
	BallModel* bmodel = (BallModel*) model;
	std::vector<std::string> res;
	for (std::vector<Model>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++)
		res.push_back((*it).geoId);
	return res;
}

void BallBallStrategy::clear() {
	// Nothing to clear
}
