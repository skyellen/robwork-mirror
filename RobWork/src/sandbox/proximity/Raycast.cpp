/*
 * Raycast.cpp
 *
 *  Created on: Apr 22, 2009
 *      Author: jimali
 */

#include "Raycast.hpp"

#define TRI_WIDTH 0.00001

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::common;


Raycast::Raycast(std::vector<rw::kinematics::Frame*> frames,
				 rw::proximity::CollisionStrategyPtr cdstrategy,
				 double ray_length):
					 _frames(frames),
					 _cdstrategy(cdstrategy),
					 // create a long triangle used as a ray
					 _ray( ownedPtr(new PlainTriMeshF(1)) )
{
	(*_ray)[0] = Triangle<float>( Vector3D<float>(0,-TRI_WIDTH,0),Vector3D<float>(0,TRI_WIDTH,0),Vector3D<float>(ray_length,0,0) );
	_rayModel = _cdstrategy->createModel();

	Geometry geom( _ray ); // we have to wrap the trimesh in an geom object
	geom.setId("Ray");
	_rayModel->addGeometry(geom);
	_cdstrategy->setFirstContact(false);
}

/*
Raycast::Raycast(rw::proximity::CollisionDetectorPtr cdetect, double ray_length):
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

Raycast::~Raycast(){}


std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >
	shoot(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& direction)
{
	// TODO: check collision between


}
