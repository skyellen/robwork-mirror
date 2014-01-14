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

#include "CircularPiHParameterization.hpp"

#include <rw/common/PropertyMap.hpp>

using namespace rw::common;
using namespace rwlibs::assembly;

CircularPiHParameterization::CircularPiHParameterization(const PropertyMap::Ptr map):
	holeRadius(map->get<double>("HoleRadius")),
	holeLength(map->get<double>("HoleLength")),
	pegRadius(map->get<double>("PegRadius")),
	pegLength(map->get<double>("PegLength")),
	angle(map->get<double>("Angle",0)),
	distanceA(map->get<double>("DistanceA",0)),
	distanceB(map->get<double>("DistanceB",0))
{
}

CircularPiHParameterization::CircularPiHParameterization(double holeRadius, double holeLength, double pegRadius, double pegLength, double angle, double distA, double distB):
	holeRadius(holeRadius),
	holeLength(holeLength),
	pegRadius(pegRadius),
	pegLength(pegLength),
	angle(angle),
	distanceA(distA),
	distanceB(distB)
{
}

CircularPiHParameterization::~CircularPiHParameterization() {
}

PropertyMap::Ptr CircularPiHParameterization::toPropertyMap() const {
	PropertyMap::Ptr map = ownedPtr(new PropertyMap());
	map->set("HoleRadius",holeRadius);
	map->set("HoleLength",holeLength);
	map->set("PegRadius",pegRadius);
	map->set("PegLength",pegLength);
	map->set("Angle",angle);
	map->set("DistanceA",distanceA);
	map->set("DistanceB",distanceB);
	return map;
}

AssemblyParameterization::Ptr CircularPiHParameterization::clone() const {
	return ownedPtr(new CircularPiHParameterization(holeRadius,holeLength,pegRadius,pegLength,angle,distanceA,distanceB));
}
