/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "DrawableModelInfo.hpp"

using namespace rw::models;

namespace {

	void init(double &scale, bool &wire, bool &high){
		scale = 1.0;
		wire = false;
		high = false;
	}

}

DrawableModelInfo::DrawableModelInfo(const std::string& id):
	_drawableId(id),
	_transform(rw::math::Transform3D<>::identity())
{
	init(_geoScale,_wireMode,_highlighted);
}

DrawableModelInfo::DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d):
	_drawableId(id),
	_transform(t3d)
{
	init(_geoScale,_wireMode,_highlighted);
}

DrawableModelInfo::DrawableModelInfo(const std::string& id, rw::math::Transform3D<> t3d,
			 double scale, bool wire, bool high):
	_drawableId(id),
	_transform(t3d),
	_geoScale(scale),
	_wireMode(wire),
	_highlighted(high)
{

}
