/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_POLYGONUTIL_HPP_
#define RW_GEOMETRY_POLYGONUTIL_HPP_

/**
 * @file PolygonUtil.hpp
 *
 * \copydoc rw::geometry::PolygonUtil
 */

#include "Polygon.hpp"

#include <rw/math/Vector2D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief Utility functions for operations on polygons, such as convex partitioning.
 *
 * The algorithm for convex partitioning of polygons has time complexity \f$ O(n r^2 \log r) \f$ where n is the number of vertices
 * and r is the number of reflex vertices (vertices that gives an inward notch). The algorithm is due to J. Mark Keil [1]. For more information, see also [2] and [3].
 * The polygons must not contain holes, and no new vertices are introduced (no Steiner points).
 *
 * [1] Minimum Decompostions of Polygonal Objects. J. Mark Keil. 1985.
 *
 * [2] http://cgm.cs.mcgill.ca/~athens/cs644/Projects/2004/LiliSang-YunjunLiu/project/MAIN3.htm
 *
 * [3] On the time bound for convex decomposition of simple polygons. Mark Keil & Jack Snoeyink. 1998. 10th Canadian Conference on Computational Geometry, Aug 1998.
 * See http://www.cs.ubc.ca/~snoeyink/convdecomp for full version paper.
 */
class PolygonUtil {
public:
	/**
	 * @brief Convex decomposition of a polygon.
	 * @param polygon [in] the polygon to decompose into convex subpolygons.
	 * @return a vector of indexed polygons. Each indexed polygon is returned as an ordered vector of indices.
	 */
	static std::vector<std::vector<std::size_t> > convexDecompositionIndexed(const Polygon<rw::math::Vector2D<> >& polygon);

	/**
	 * @brief Convex decomposition of a polygon.
	 * @param polygon [in] the polygon to decompose into convex subpolygons.
	 * @return a vector of convex polygons.
	 */
	static std::vector<Polygon<rw::math::Vector2D<> > > convexDecomposition(const Polygon<rw::math::Vector2D<> >& polygon);

	/**
	 * @brief Check if point lies inside convex polygon.
	 * @param point [in] the point.
	 * @param polygon [in] the polygon.
	 * @param eps [in] distance threshold for when to consider a point inside the polygon.
	 * @return true if \b point is strictly inside the \b polygon, or false if on the border or outside.
	 */
	static bool isInsideConvex(const rw::math::Vector2D<>& point, const Polygon<rw::math::Vector2D<> >& polygon, double eps);

	/**
	 * @brief Get the signed area of a 2D polygon.
	 * @param polygon [in] the polygon to find area of.
	 * @return area of the polygon, with negative sign if vertices are given clockwise, or positive sign if given counter-clockwise.
	 */
	static double area(const Polygon<rw::math::Vector2D<> >& polygon);

private:
	PolygonUtil();
	virtual ~PolygonUtil();
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_POLYGONUTIL_HPP_ */
