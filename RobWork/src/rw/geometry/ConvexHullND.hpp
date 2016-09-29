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

#ifndef RW_GEOMETRY_CONVEXHULLHULLND_HPP_
#define RW_GEOMETRY_CONVEXHULLHULLND_HPP_

#include <vector>
#include <rw/math/VectorND.hpp>
#include <rw/common/Ptr.hpp>

//! @file ConvexHullND.hpp

namespace rw {
namespace geometry {
//! @addtogroup geometry
// @{


	/**
	 * @brief interface for convexhull calculators on 3d point sets
	 */
    template<size_t N>
	class ConvexHullND {
	public:
	    //! smart pointer type of this class
	    typedef rw::common::Ptr<ConvexHullND<N> > Ptr;

		/**
		 * @brief rebuilts the hull
		 * @param vertices
		 */
		virtual void rebuild(const std::vector<rw::math::VectorND<N> >& vertices) = 0;

		/**
		 * @brief test if the given vertex is inside the convex hull
		 */
		//virtual bool isInside(const rw::math::VectorND<N>& vertex, const std::vector<rw::math::VectorND<N> >& vertices) = 0;
		virtual bool isInside(const rw::math::VectorND<N>& vertex) = 0;

		/**
		 * @brief If the vertex is inside the convex hull the minimum distance
		 * to any of the half-spaces of the hull is returned. If its not inside
		 * 0 is returned.
		 * @param vertex
		 * @return
		 */
		//virtual double getMinDistInside(const rw::math::VectorND<N>& vertex, const std::vector<rw::math::VectorND<N> >& vertices) = 0;
		virtual double getMinDistInside(const rw::math::VectorND<N>& vertex) = 0;
		
		/**
		 * @brief Calculate average distance from point inside the convex hull to its walls.
		 * 
		 * Each distance is weighted according to the wall's volume.'
		 */
		virtual double getAvgDistInside(const rw::math::VectorND<N>& vertex) = 0;

		/**
		 * @brief If the vertex is outside the convex hull the minimum distance
		 * to the convex hull is returned. If its not outside 0 is returned.
		 * @param vertex
		 * @return
		 */
		virtual double getMinDistOutside(const rw::math::VectorND<N>& vertex) = 0;
		
		/**
		 * @brief Return centroid of the convex hull.
		 * 
		 * Centroid is calculated as an average of face centroids weighted by face area.
		 */
		virtual rw::math::VectorND<N> getCentroid() = 0;

		/**
		 * @brief Returns the point on the convex hull closest to the \b vertex .
		 * @param vertex [in] the vertex.
		 * @return the closest point.
		 */
		virtual rw::math::VectorND<N> getClosestPoint(const rw::math::VectorND<N>& vertex) = 0;

    };

	//! @}

} //end namespace geometry
} //end namespace rw

#endif /* HULL3D_HPP_ */
