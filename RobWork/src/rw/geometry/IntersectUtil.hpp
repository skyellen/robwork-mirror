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

#ifndef RW_GEOMETRY_INTERSECTUTIL_HPP_
#define RW_GEOMETRY_INTERSECTUTIL_HPP_

/**
 * @file
 * @copydoc rw::geometry::IntersectUtil
 */

#include <rw/math/Vector3D.hpp>
#include "Triangle.hpp"

namespace rw {
namespace geometry {
	class Plane;
	class Line;

	//! @addtogroup geometry
	// @{
    //! @file rw/geometry/IntersectUtil.hpp
	/**
	 * @brief utility class for calculating intersection points between geometry primitives
	 */
	class IntersectUtil {
	public:
		/// closest point tests

		/**
		 * @brief finds the point on a line which is closest to the point \b point
		 */
		static
		rw::math::Vector3D<> closestPt(const rw::math::Vector3D<>& point, const Line& line);


		/**
		 * @brief finds the point on a ray (infinite in all directions) which is
		 * closest to the point \b point
		 */
		static
		rw::math::Vector3D<> closestPtPointRay(const rw::math::Vector3D<>& point,
									   const rw::math::Vector3D<>& p1,
									   const rw::math::Vector3D<>& p2);

		/**
		 * @brief finds the point on a line which is closest to the point \b point
		 */
		static
		rw::math::Vector3D<> closestPtPointLine(const rw::math::Vector3D<>& point,
									   const rw::math::Vector3D<>& p1,
									   const rw::math::Vector3D<>& p2);

		/**
		 * @brief finds the point on a line which is closest to the point \b point
		 */
		/*static rw::math::Vector3D<> closestPtPointRay(const rw::math::Vector3D<>& point,
			      							          const rw::math::Vector3D<>& ray);
*/
		/// closest point pair tests

		/**
		 * @brief finds the closest points between two line segments
		 */
		//static PointPair closestPts(const Line& lineA, const Line& lineB);

		/**
		 * @brief finds the closest points between a line segment and a triangle
		 */
		//static PointPair closestPts(const Line& line, const Triangle<>& tri);

		/**
		 * @brief finds the closest points between a ray and a triangle
		 */
		//static PointPair closestPtsRayTraingle(const rw::math::Vector3D<>& ray, const Triangle<>& tri);


		/// intersection tests



		/**
		 * @brief calculates the intersection point between the ray and the plane.
		 * If they intersect true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtRayPlane(
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const Plane& p,
								rw::math::Vector3D<>& dst);

		/**
		 * @brief calculates the intersection point between the ray and the implicit
		 * plane defined by the three points \b p1 \b p2 \b p3.
		 * If they intersect, true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtRayPlane(const rw::math::Vector3D<>& ray1,
								const rw::math::Vector3D<>& ray2,
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const rw::math::Vector3D<>& p3,
								rw::math::Vector3D<>& dst);

		/**
		 * @brief calculates the intersection point between the ray and the triangle.
		 * If they intersect true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtRayTri(
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const Triangle<>& tri,
								rw::math::Vector3D<>& dst);

		/**
		 * @brief calculates the intersection point between the ray and the
		 * triangle defined by the three points \b p1 \b p2 \b p3.
		 * If they intersect, true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtRayTri(const rw::math::Vector3D<>& ray1,
								const rw::math::Vector3D<>& ray2,
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const rw::math::Vector3D<>& p3,
								rw::math::Vector3D<>& dst);



		/**
		 * @brief calculates the intersection point between the ray and the plane.
		 * If they intersect true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtLinePlane(
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const Plane& p,
								rw::math::Vector3D<>& dst);

		/**
		 * @brief calculates the intersection point between the ray and the implicit
		 * plane defined by the three points \b p1 \b p2 \b p3.
		 * If they intersect, true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtLinePlane(const rw::math::Vector3D<>& ray1,
								const rw::math::Vector3D<>& ray2,
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const rw::math::Vector3D<>& p3,
								rw::math::Vector3D<>& dst);


		/**
		 * @brief calculates the intersection point between the ray and the triangle.
		 * If they intersect true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtLineTri(
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const Triangle<>& tri,
								rw::math::Vector3D<>& dst);

		/**
		 * @brief calculates the intersection point between the line segment and the
		 * triangle defined by the three points \b p1 \b p2 \b p3.
		 * If they intersect, true is returned and intersection point is set in \b dst
		 */
		static
		bool intersetPtLineTri(const rw::math::Vector3D<>& ray1,
								const rw::math::Vector3D<>& ray2,
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const rw::math::Vector3D<>& p3,
								rw::math::Vector3D<>& dst);





		/**
		 * @brief calculates the intersection point between the triangle \b triA and the triangle \b triB.
		 * If they intersect, true is returned and two intersection points are set in \b dst1 and \b dst2
		 *
		 * @note this is not an optimized method
		 */
		static
		bool intersetPtTriTri(
								const Triangle<>& triA,
								const Triangle<>& triB,
								rw::math::Vector3D<>& dst1,
								rw::math::Vector3D<>& dst2);


	};

	// @}
} // geometry
} // rw


#endif /* INTERSECTUTIL_HPP_ */
