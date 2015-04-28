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

#include "TNTFrictionModelMicroSlip.hpp"
#include "TNTFrictionModelData.hpp"
#include "TNTContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::tntphysics;

class TNTFrictionModelMicroSlip::Data: public TNTFrictionModelData {
public:
	Data(): grossData(NULL), xhi(Vector3D<>::zero()) {}
	virtual ~Data() {
		if (grossData != NULL)
			delete grossData;
	}
	TNTFrictionModelData* clone() {
		Data* const clone = new Data();
		clone->xhi = xhi;
		if (grossData != NULL)
			clone->grossData = grossData->clone();
		return clone;
	}
	TNTFrictionModelData* grossData;
	Vector3D<> xhi;
};

TNTFrictionModelMicroSlip::TNTFrictionModelMicroSlip():
	_gamma(0),
	_r(0),
	_grossModel(NULL),
	_ownsGrossModel(false)
{
}

TNTFrictionModelMicroSlip::TNTFrictionModelMicroSlip(const PropertyMap &map):
	_gamma(map.get<double>("gamma",0)),
	_r(map.get<double>("r",0)),
	_grossModel(TNTFrictionModel::Factory::makeModel(map.get<std::string>("GrossModel","Coulomb"),map)),
	_ownsGrossModel(true)
{
}

TNTFrictionModelMicroSlip::TNTFrictionModelMicroSlip(double gamma, double r, const TNTFrictionModel* grossModel):
	_gamma(gamma),
	_r(r),
	_grossModel(grossModel),
	_ownsGrossModel(false)
{
}

TNTFrictionModelMicroSlip::~TNTFrictionModelMicroSlip() {
	if (_ownsGrossModel && _grossModel != NULL)
		delete _grossModel;
}

const TNTFrictionModel* TNTFrictionModelMicroSlip::withProperties(const PropertyMap &map) const {
	if (!map.has("gamma"))
		RW_THROW("TNTFrictionModelMicroSlip (withProperties): could not create model as property \"gamma\" was not found in map.");
	if (!map.has("r"))
		RW_THROW("TNTFrictionModelMicroSlip (withProperties): could not create model as property \"r\" was not found in map.");
	if (!map.has("GrossModel"))
		RW_THROW("TNTFrictionModelMicroSlip (withProperties): could not create model as property \"GrossModel\" was not found in map.");
	return new TNTFrictionModelMicroSlip(map);
}

TNTFrictionModelData* TNTFrictionModelMicroSlip::makeDataStructure() const {
	Data* const data = new Data();
	data->grossData = _grossModel->makeDataStructure();
	return data;
}

void TNTFrictionModelMicroSlip::updateData(
	const TNTContact& contact,
	const TNTIslandState& tntstate,
	const State& rwstate,
	double h,
	TNTFrictionModelData* data) const
{
	if (data == NULL)
		RW_THROW("TNTFrictionModelMicroSlip (updateData): data was NULL!");
	Data* const intData = dynamic_cast<Data*>(data);
	if (intData == NULL)
		RW_THROW("TNTFrictionModelMicroSlip (updateData): data was not of correct type!");
	_grossModel->updateData(contact,tntstate,rwstate,h,intData->grossData);

	static const double eps = 1e-6;

	// Find the current relative velocity:
	const Vector3D<> n = contact.getNormalW(tntstate);
	const Vector3D<> velP = contact.getVelocityParentW(tntstate,rwstate).linear();
	const Vector3D<> velC = contact.getVelocityChildW(tntstate,rwstate).linear();
	Vector3D<> relVel = velP-velC;
	relVel = relVel - dot(relVel,n)*n;
	const double relVelSize = relVel.norm2();

	// Project xhi
	const Vector3D<> xhi = intData->xhi - dot(intData->xhi,n)*n;

	// Determine current direction vector
	Vector3D<> dir;
	if (relVelSize > eps)
		dir = relVel/relVelSize;
	else if (xhi.norm2() > eps)
		dir = normalize(xhi);
	else
		dir = Vector3D<>::zero();

	// Now find new xhi
	const Vector3D<> xhiPar = dir*(1-(1-dot(xhi,dir))/(relVelSize+exp(-relVelSize))*exp(-h*(_gamma*relVelSize+_r)));
	const Vector3D<> xhiPerp = (xhi-dot(xhi,dir)*dir)*exp(-h*(_gamma*relVelSize+_r));
	intData->xhi = xhiPar+xhiPerp;
}

TNTFrictionModel::DryFriction TNTFrictionModelMicroSlip::getDryFriction(
	const TNTContact& contact,
	const TNTIslandState& tntstate,
	const State& rwstate,
	const TNTFrictionModelData* data) const
{
	if (data == NULL)
		RW_THROW("TNTFrictionModelMicroSlip (getDryFriction): data was NULL!");
	const Data* const intData = dynamic_cast<const Data*>(data);
	if (intData == NULL)
		RW_THROW("TNTFrictionModelMicroSlip (getDryFriction): data was not of correct type!");
	const TNTFrictionModelData* const grossData = intData->grossData;

	const DryFriction fgVal = _grossModel->getDryFriction(contact,tntstate,rwstate,grossData);
	DryFriction res;
	res.enableTangent = true;
	res.tangent = fgVal.tangent*intData->xhi.norm2();
	res.tangentDirection = normalize(intData->xhi);
	return res;
}

Wrench6D<> TNTFrictionModelMicroSlip::getViscuousFriction(
	const TNTContact& contact,
	const TNTIslandState& tntstate,
	const State& rwstate,
	const TNTFrictionModelData* data) const
{
	return Wrench6D<>(Vector3D<>::zero(),Vector3D<>::zero());
}
