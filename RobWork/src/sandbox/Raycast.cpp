/*
 * Raycast.cpp
 *
 *  Created on: Apr 22, 2009
 *      Author: jimali
 */

#include "Raycast.hpp"

#define TRI_LENGTH 0.00001

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::geometry;

Raycast::Raycast(Frame *rayFrame,
                 std::vector<rw::kinematics::Frame*> frames,
				 rw::proximity::CollisionStrategyPtr cdstrategy):
					 _frames(frames),
					 _cdstrategy(cdstrategy),
					 // create a long triangle used as a ray
					 _ray(Vector3D<>(0,-TRI_LENGTH,0),Vector3D<>(0,TRI_LENGTH,0),Vector3D<>(1000,0,0))
{
    std::vector<rw::geometry::Face<> > ray;
    ray.push_back(_ray);
	cdstrategy->addModel(rayFrame, ray);
	cdstrategy->setFirstContact(false);
}

Raycast::~Raycast(){}


std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >
	shoot(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& direction)
{

	// check the ray against all frames
    BOOST_FOREACH(Frame *obj, _frames){
        _cdstrategy->inCollision();
    }
}
