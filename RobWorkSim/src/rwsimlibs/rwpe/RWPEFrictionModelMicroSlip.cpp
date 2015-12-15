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

#include "RWPEFrictionModelData.hpp"
#include "RWPEFrictionModelMicroSlip.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

class RWPEFrictionModelMicroSlip::Data: public RWPEFrictionModelData {
public:
	Data(): grossData(NULL), xhi(Vector3D<>::zero()) {}
	virtual ~Data() {
		if (grossData != NULL)
			delete grossData;
	}
	RWPEFrictionModelData* clone() {
		Data* const clone = new Data();
		clone->xhi = xhi;
		if (grossData != NULL)
			clone->grossData = grossData->clone();
		return clone;
	}
	RWPEFrictionModelData* grossData;
	Vector3D<> xhi;
};

RWPEFrictionModelMicroSlip::RWPEFrictionModelMicroSlip():
	_gamma(0),
	_r(0),
	_grossModel(NULL),
	_ownsGrossModel(false)
{
}

RWPEFrictionModelMicroSlip::RWPEFrictionModelMicroSlip(const PropertyMap &map):
	_gamma(map.get<double>("gamma",0)),
	_r(map.get<double>("r",0)),
	_grossModel(RWPEFrictionModel::Factory::makeModel(map.get<std::string>("GrossModel","Coulomb"),map)),
	_ownsGrossModel(true)
{
}

RWPEFrictionModelMicroSlip::RWPEFrictionModelMicroSlip(double gamma, double r, const RWPEFrictionModel* grossModel):
	_gamma(gamma),
	_r(r),
	_grossModel(grossModel),
	_ownsGrossModel(false)
{
}

RWPEFrictionModelMicroSlip::~RWPEFrictionModelMicroSlip() {
	if (_ownsGrossModel && _grossModel != NULL)
		delete _grossModel;
}

const RWPEFrictionModel* RWPEFrictionModelMicroSlip::withProperties(const PropertyMap &map) const {
	if (!map.has("gamma"))
		RW_THROW("RWPEFrictionModelMicroSlip (withProperties): could not create model as property \"gamma\" was not found in map.");
	if (!map.has("r"))
		RW_THROW("RWPEFrictionModelMicroSlip (withProperties): could not create model as property \"r\" was not found in map.");
	if (!map.has("GrossModel"))
		RW_THROW("RWPEFrictionModelMicroSlip (withProperties): could not create model as property \"GrossModel\" was not found in map.");
	return new RWPEFrictionModelMicroSlip(map);
}

RWPEFrictionModelData* RWPEFrictionModelMicroSlip::makeDataStructure() const {
	Data* const data = new Data();
	data->grossData = _grossModel->makeDataStructure();
	return data;
}

void RWPEFrictionModelMicroSlip::updateData(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	double h,
	RWPEFrictionModelData* data) const
{
	if (data == NULL)
		RW_THROW("RWPEFrictionModelMicroSlip (updateData): data was NULL!");
	Data* const intData = dynamic_cast<Data*>(data);
	if (intData == NULL)
		RW_THROW("RWPEFrictionModelMicroSlip (updateData): data was not of correct type!");
	_grossModel->updateData(contact,islandState,rwstate,h,intData->grossData);

	static const double eps = 1e-6;

	// Find the current relative velocity:
	const Vector3D<> n = contact.getNormalW(islandState);
	const Vector3D<> velP = contact.getVelocityParentW(islandState,rwstate).linear();
	const Vector3D<> velC = contact.getVelocityChildW(islandState,rwstate).linear();
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

RWPEFrictionModel::DryFriction RWPEFrictionModelMicroSlip::getDryFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	if (data == NULL)
		RW_THROW("RWPEFrictionModelMicroSlip (getDryFriction): data was NULL!");
	const Data* const intData = dynamic_cast<const Data*>(data);
	if (intData == NULL)
		RW_THROW("RWPEFrictionModelMicroSlip (getDryFriction): data was not of correct type!");
	const RWPEFrictionModelData* const grossData = intData->grossData;

	const DryFriction fgVal = _grossModel->getDryFriction(contact,islandState,rwstate,grossData);
	DryFriction res;
	res.enableTangent = true;
	res.tangent = fgVal.tangent*intData->xhi.norm2();
	res.tangentDirection = normalize(intData->xhi);
	return res;
}

Wrench6D<> RWPEFrictionModelMicroSlip::getViscuousFriction(
	const RWPEContact& contact,
	const RWPEIslandState& islandState,
	const State& rwstate,
	const RWPEFrictionModelData* data) const
{
	return Wrench6D<>(Vector3D<>::zero(),Vector3D<>::zero());
}
