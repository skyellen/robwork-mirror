/*
 * Raycast.hpp
 *
 *  Created on: Apr 22, 2009
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_RAYCAST_HPP_
#define RW_PROXIMITY_RAYCAST_HPP_

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/math/Vector3D.hpp>


namespace rw {
namespace proximity {

	/**
	 * @brief a raycast implementation that relies on a collision detector for finding the
	 * collision between the ray and the scene.
	 */
	class Raycast {
	public:

		/**
		 * @brief constructor
		 *
		 */
		//Raycast(rw::proximity::CollisionDetectorPtr cdetector, double ray_length=100.0);

		Raycast(std::vector<rw::kinematics::Frame*> frames,
						 rw::proximity::CollisionStrategyPtr cdstrategy,
						 double ray_length=100.0);


		//! @brief destructor
		virtual ~Raycast();

		/**
		 * @brief shoots a ray in the direction of the vector \b direction starting from
		 * the position vector \b pos. The frame associated with the geometry that is hit
		 * first by the ray is returned along with the intersection point described in
		 * world frame.
		 */
		std::pair<rw::kinematics::Frame*,rw::math::Vector3D<> >
			shoot(const rw::math::Vector3D<>& pos,
				  const rw::math::Vector3D<>& direction,
				  const rw::kinematics::State& state);

	private:
		//rw::proximity::CollisionDetectorPtr _detector;
		std::vector<rw::kinematics::Frame*> _frames;
		rw::proximity::CollisionStrategyPtr _cdstrategy;
		rw::geometry::PlainTriMeshFPtr _ray;
		rw::kinematics::Frame *_rayFrame;

		rw::proximity::ProximityModelPtr _rayModel;
	};

}
}

#endif /* RAYCAST_HPP_ */
