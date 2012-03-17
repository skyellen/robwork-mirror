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

#ifndef RW_PROXIMITY_RAYCASTER_HPP_
#define RW_PROXIMITY_RAYCASTER_HPP_

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
	class Raycaster {
	public:

		/**
		 * @brief constructor
		 *
		 */
		//Raycast(rw::proximity::CollisionDetectorPtr cdetector, double ray_length=100.0);

		Raycaster(std::vector<rw::kinematics::Frame*> frames,
						 rw::proximity::CollisionStrategy::Ptr cdstrategy,
						 double ray_length=100.0);


		//! @brief destructor
		virtual ~Raycaster();

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
		rw::proximity::CollisionStrategy::Ptr _cdstrategy;
		rw::geometry::PlainTriMeshF::Ptr _ray;
		rw::kinematics::Frame *_rayFrame;

		rw::proximity::ProximityModel::Ptr _rayModel;
	};

}
}

#endif /* RAYCAST_HPP_ */
