/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "PiHParameterization.hpp"

using namespace rw::common;
using rw::math::Q;
using namespace rwlibs::assembly;

PiHParameterization::PiHParameterization(PropertyMap::Ptr pmap) {
	reset(pmap);
}

PiHParameterization::PiHParameterization(double holeRadius,
		double pegRadius,
		double theta,
		double phi,
		double distX,
		double distY,
		double distTContact,
		double x0
	):
	holeRadius(holeRadius), pegRadius(pegRadius), theta(theta), phi(phi), distX(distX), distY(distY), distTContact(distTContact), x0(x0)
{
}

PiHParameterization::~PiHParameterization() {
}

PropertyMap::Ptr PiHParameterization::toPropertyMap() const {
	PropertyMap::Ptr map = ownedPtr(new PropertyMap());
	if (!map->has("HoleRadius"))
		map->add("HoleRadius","Radius of hole (in millimeters).",0.);
	if (!map->has("PegRadius"))
		map->add("PegRadius","Radius of peg (in millimeters).",0.);
	if (!map->has("Theta"))
		map->add("Theta","Angle (in radians).",0.);
	if (!map->has("Phi"))
		map->add("Phi","Angle (in radians).",0.);
	if (!map->has("DistX"))
		map->add("DistX","Distance in x (in millimeters).",0.);
	if (!map->has("DistY"))
		map->add("DistY","Distance in y (in millimeters).",0.);
	if (!map->has("DistTContact"))
		map->add("DistTContact","Distance to contact (in millimeters).",0.);
	if (!map->has("x0"))
		map->add("x0","Distance (in millimeters).",0.);
	map->set("HoleRadius",holeRadius*1000.);
	map->set("PegRadius",pegRadius*1000.);
	map->set("Theta",theta);
	map->set("Phi",phi);
	map->set("DistX",distX*1000.);
	map->set("DistY",distY*1000.);
	map->set("DistTContact",distTContact*1000.);
	map->set("x0",x0*1000.);
	return map;
}

AssemblyParameterization::Ptr PiHParameterization::clone() const {
	return ownedPtr(new PiHParameterization(holeRadius, pegRadius, theta, phi, distX, distY, distTContact, x0));
}

void PiHParameterization::reset(PropertyMap::Ptr pmap) {
	AssemblyParameterization::reset(pmap);
	holeRadius = pmap->get<double>("HoleRadius",1.)/1000.;
	pegRadius = pmap->get<double>("PegRadius",1.)/1000.;
	theta = pmap->get<double>("Theta",0.);
	phi = pmap->get<double>("Phi",0.);
	distX = pmap->get<double>("DistX",0.)/1000.;
	distY = pmap->get<double>("DistY",0.)/1000.;
	distTContact = pmap->get<double>("DistTContact",0.)/1000.;
	x0 = pmap->get<double>("x0",0.)/1000.;
}
