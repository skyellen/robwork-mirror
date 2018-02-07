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

#ifndef RW_GEOMETRY_DELAUNAY_HPP_
#define RW_GEOMETRY_DELAUNAY_HPP_

/**
 * @file Delaunay.hpp
 *
 * \copydoc rw::geometry::Delaunay
 */

#include "IndexedTriMesh.hpp"

#include <rw/math/Vector2D.hpp>

#include <vector>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief Utility functions for doing Delaunay triangulations.
 */
class Delaunay {
public:
	/**
	 * @brief Do the Delaunay triangulation of a set of 2D points.
	 *
	 * The border of the triangulation will be the convex hull of the vertices.
	 * It is possible to attach a value to each of the vertices, which will be the third coordinate in the returned 3D triangle mesh.
	 * If no values are given, the third coordinate will simply be zero.
	 *
	 * @param vertices [in] the set of 2D points to triangulate.
	 * @param values [in] (optional) attach a value to each of the vertices.
	 * @return an indexed triangle mesh in 3D, where the first two coordinates gives the triangulation and the third coordinate holds corresponding \b values if given.
	 */
	static rw::geometry::IndexedTriMesh<>::Ptr triangulate(const std::vector<rw::math::Vector2D<> >& vertices, const std::vector<double>& values = std::vector<double>());

private:
	Delaunay();
	virtual ~Delaunay();
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_DELAUNAY_HPP_ */
