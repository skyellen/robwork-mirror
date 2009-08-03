/*
 * Raycast.hpp
 *
 *  Created on: Apr 22, 2009
 *      Author: jimali
 */

#ifndef RAYCAST_HPP_
#define RAYCAST_HPP_

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/math/Vector3D.hpp>

/**
 * @brief
 */
class Raycast {
public:

	Raycast(rw::kinematics::Frame *rayFrame,
	        std::vector<rw::kinematics::Frame*> frames,
	        rw::proximity::CollisionStrategyPtr cdstrategy);

	virtual ~Raycast();

	/**
	 * @brief shoots a ray in the direction of the vector \b direction starting from
	 * the position vector \b pos. The frame associated with the geometry that is hit
	 * first by the ray is returned along with the intersection point described in
	 * world frame.
	 */
	std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >
		shoot(const rw::math::Vector3D<>& pos, const rw::math::Vector3D<>& direction);

private:
	rw::proximity::CollisionStrategyPtr _cdstrategy;
	rw::geometry::Face<> _ray;
	rw::kinematics::Frame *_rayFrame;
};


#endif /* RAYCAST_HPP_ */
