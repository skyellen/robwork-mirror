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

#ifndef RW_GEOMETRY_GIFTWRAPHULL3D_HPP_
#define RW_GEOMETRY_GIFTWRAPHULL3D_HPP_

#include "ConvexHull3D.hpp"
#include "Triangle.hpp"
#include "PlainTriMesh.hpp"

#include <stack>
#include <set>
#include <vector>



namespace rw {
namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file GiftWrapHull.hpp
     */

    /**
	 * @brief calculates the convex hull of a set of 3d points.
	 *
	 * The GirftWrap convex hull algorithm is used, hence the
	 * class name.
	 */
	class GiftWrapHull3D : public ConvexHull3D {
	public:
		/**
		 * @brief constructor
		 */
		GiftWrapHull3D(){};

		/**
		 * @brief destructor
		 */
		virtual ~GiftWrapHull3D(){};

		//! @copydoc ConvexHull3D::rebuild
		void rebuild(const std::vector<rw::math::Vector3D<> >& vertices);

		//! @copydoc ConvexHull3D::isInside
		bool isInside(const rw::math::Vector3D<>& vertex);

		//! @copydoc ConvexHull3D::getMinDist
		double getMinDist(const rw::math::Vector3D<>& vertex);

		//! @copydoc ConvexHull3D::toTriMesh
		rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<double> >* toTriMesh();

	public:
		typedef std::pair<int,int> EdgeIdx;

		struct TriangleIdx {
			TriangleIdx(int v1, int v2, int v3, const rw::math::Vector3D<>& n):
				_n(n)
			{
				_vIdx[0]=v1;
				_vIdx[1]=v2;
				_vIdx[2]=v3;
			}
			int _vIdx[3];
			rw::math::Vector3D<> _n;
		};

	private:


		int search(const EdgeIdx& edge, const rw::math::Vector3D<>& normal, std::vector<int> &candIdxs);

	private:
		// vertices on the hull
		std::vector<rw::math::Vector3D<> > _vertices;
		// edges between vertices
		std::set<EdgeIdx> _edgeSet;
		std::stack<std::pair<EdgeIdx,int> > _edgeStack;
		// triangles composed of edges
		std::vector<TriangleIdx> _tris;
	};
	//! @}
}
}
#endif /* GIFTWRAPHULL_HPP_ */
