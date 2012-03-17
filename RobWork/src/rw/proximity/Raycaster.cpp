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

#define TRI_WIDTH 0.00001

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::common;


Raycaster::Raycaster(std::vector<rw::kinematics::Frame*> frames,
				 rw::proximity::CollisionStrategy::Ptr cdstrategy,
				 double ray_length):
					 _frames(frames),
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


std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >
Raycaster::shoot(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& direction,const rw::kinematics::State& state)
{
	// TODO: check collision between
    return std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >((rw::kinematics::Frame*)NULL, Vector3D<>(0,0,0));

}
