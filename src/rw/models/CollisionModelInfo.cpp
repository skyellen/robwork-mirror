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

#include "CollisionModelInfo.hpp"

using namespace rw::models;

CollisionModelInfo::CollisionModelInfo(const std::string& id, double scale):
	_colId(id),
	_transform(rw::math::Transform3D<>::identity()),
	_geoScale(scale)
{
}

CollisionModelInfo::CollisionModelInfo(const std::string& id,
			rw::math::Transform3D<> t3d, double scale):
	_colId(id),
	_transform(t3d),
	_geoScale(scale)
{
}
