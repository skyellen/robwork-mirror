/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#include "Proximity.hpp"

#include <rw/models/Models.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/ConveyorItem.hpp>
#include <rw/models/VirtualJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>
#include <map>

using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::models;
using namespace rw::geometry;



CollisionSetup Proximity::getCollisionSetup(const WorkCell& workcell)
{
    return CollisionSetup::get(workcell);
}

std::vector<Geometry::Ptr> Proximity::getGeometry(const rw::kinematics::Frame* frame){
	std::vector<Geometry::Ptr> geoms;

	std::vector<CollisionModelInfo> modelInfos = CollisionModelInfo::get(frame);
	if( modelInfos.size()==0 ){
		return geoms;
	}

	BOOST_FOREACH(CollisionModelInfo &info, modelInfos){
		Geometry::Ptr geom = GeometryFactory::getGeometry(info.getGeoString());
		if(geom==NULL)
			continue;

		geom->setTransform( info.getTransform() );
		geom->setScale( info.getGeoScale() );
		geoms.push_back(geom);
	}
	return geoms;
}

