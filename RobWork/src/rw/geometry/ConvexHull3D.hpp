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

#ifndef RW_GEOMETRY_CONVEXHULLHULL3D_HPP_
#define RW_GEOMETRY_CONVEXHULLHULL3D_HPP_

//! @file ConvexHull3D.hpp

#include "Triangle.hpp"
#include "PlainTriMesh.hpp"

#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry
// @{


	/**
	 * @brief interface for convexhull calculators on 3d point sets
	 */
	class ConvexHull3D {
	public:
	    //! smart pointer type of this class
	    typedef rw::common::Ptr<ConvexHull3D> Ptr;

	    //! destructor
	    virtual ~ConvexHull3D(){};

	    /**
		 * @brief rebuilts the hull
		 * @param vertices
		 */
		virtual void rebuild(const std::vector<rw::math::Vector3D<> >& vertices) = 0;

		/**
		 * @brief test if the given vertex is inside the convex hull
		 */
		virtual bool isInside(const rw::math::Vector3D<>& vertex) = 0;

		/**
		 * @brief If the vertex is inside the convex hull the minimum distance
		 * to any of the half-spaces of the hull is returned. If its not inside
		 * 0 is returned.
		 * @param vertex
		 * @return
		 */
		virtual double getMinDistInside(const rw::math::Vector3D<>& vertex) = 0;

		/**
		 * @brief If the vertex is outside the convex hull the minimum distance
		 * to the convex hull is returned. If its not outside 0 is returned.
		 * @param vertex
		 * @return
		 */
		virtual double getMinDistOutside(const rw::math::Vector3D<>& vertex) = 0;


		/**
		 * @brief create a plain trimesh from the hull facets
		 * @return the hull facets as a plain triangle mesh with normal information
		 */
		virtual rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<> >::Ptr toTriMesh() = 0;

	};
	//! @}

} //end namespace geometry
} //end namespace rw

#endif /* HULL3D_HPP_ */
