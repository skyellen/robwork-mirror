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

#include "Raycaster.hpp"

#include <rw/geometry/Geometry.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

#define TRI_WIDTH 0.00001

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::common;

/*
Raycaster::Raycaster(double ray_length=100.0):
	 // create a long triangle used as a ray
	 _ray( ownedPtr(new PlainTriMeshF(1)) )
{
	_cdstrategy = cdetector->getCollisionStrategy();
	(*_ray)[0] = Triangle<float>( Vector3D<float>(0,(float)-TRI_WIDTH,0),Vector3D<float>(0,(float)TRI_WIDTH,0),Vector3D<float>((float)ray_length,0,0) );
	_rayModel = _cdstrategy->createModel();

	Geometry geom( _ray ); // we have to wrap the trimesh in an geom object
	geom.setId("Ray");
	_rayModel->addGeometry(geom);
	//_cdstrategy->setFirstContact(false);
}
*/

Raycaster::Raycaster(
				 rw::proximity::CollisionStrategy::Ptr cdstrategy,
				 double ray_length):
					 _cdstrategy(cdstrategy),
					 // create a long triangle used as a ray
					 _ray( ownedPtr(new PlainTriMeshF(1)) )
{
	(*_ray)[0] = Triangle<float>( Vector3D<float>(0,(float)-TRI_WIDTH,0),Vector3D<float>(0,(float)TRI_WIDTH,0),Vector3D<float>((float)ray_length,0,0) );
	_rayModel = _cdstrategy->createModel();

	Geometry geom( _ray ); // we have to wrap the trimesh in an geom object
	geom.setId("Ray");
	_rayModel->addGeometry(geom);
	//_cdstrategy->setFirstContact(false);
}

/*
Raycaster::Raycaster(rw::proximity::CollisionDetectorPtr cdetect, double ray_length):
					 _detector(cdetect),
					 // create a long triangle used as a ray
					 _ray(1)
{
	_ray[0] = Triangle<float>( Vector3D<float>(0,-TRI_WIDTH,0),Vector3D<float>(0,TRI_WIDTH,0),Vector3D<float>(ray_length,0,0) );
	_rayModel = _cdstrategy->createModel();

	Geometry geom( _ray ); // we have to wrap the trimesh in an geom object
	_rayModel->addGeometry(geom);


}
*/

Raycaster::~Raycaster(){}


bool Raycaster::shoot(const rw::math::Vector3D<>& pos,
				         const rw::math::Vector3D<>& direction,
				         Raycaster::QueryResult& result,
				         const rw::kinematics::State& state)
{

	FKTable fk(state);
	Transform3D<> wTray = Transform3D<>::identity();

	if(_rayFrame)
		wTray = fk.get(_rayFrame);

	// check collision between ray and all obstacles in the scene
	double minDist = std::numeric_limits<double>::max();
	std::vector<CollisionStrategy::Contact> contacts;
	for(size_t i=0; i<_obstacles.size();i++){
		ProximityModel::Ptr object = _obstacles[i];

		if( _cdstrategy->inCollision(_rayModel, wTray, object, Transform3D<>::identity(),  result.data) ){

			// now get the contact points and their surface normals
			contacts.clear();
			_cdstrategy->getCollisionContacts(contacts, result.data);

			BOOST_FOREACH(CollisionStrategy::Contact &contact, contacts){
				double dist = MetricUtil::dist2(contact.point, pos);
				if(dist<minDist){
					result.point = contact.point;
					result.normal = contact.normalB;
				}

				if(_queryType == ALL_HITS){
					result.points.push_back(contact.point);
					result.normals.push_back(contact.normalB);
				}
			}
		}
	}

	if(minDist ==  std::numeric_limits<double>::max())
		return false;
	return true;
}
