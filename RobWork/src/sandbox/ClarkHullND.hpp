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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/function.hpp>
#include <stack>
#include <set>
#include <vector>



namespace rw {
namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file ClarkHullND.hpp
     */

    namespace clarkhull {
        /**
         * @brief calclates the convex hull of a set of vertices \b coords each with dimension  \b dim
         *
         * @param dim [in] nr of dimensions in each vertice
         * @param coords [in] array of vertices
         * @param nrCoords [in] the number of vertices
         * @param vertIdxs [out] the vertice indices of the hull
         * @param faceIdxs [out] the face indices of the hull
         * @param faceNormals [out] the face normals of the hull
         */
        void build(size_t dim, double *coords, size_t nrCoords,
                   std::vector<int>& vertIdxs,
                   std::vector<int>& faceIdxs,
                   std::vector<double>& faceNormals);
    }

    /**
	 * @brief calculates the convex hull of a set of 3d points.
	 *
	 * The GirftWrap convex hull algorithm is used, hence the
	 * class name.
	 *
	 * @note It is important that there are not multiple vertices at the same coordinates.
	 * Filter these away before using this convex hull calculation.
	 */
    template <std::size_t N>
	class ClarkHullND: public ConvexHullND<N> {
	public:
	    typedef boost::numeric::ublas::bounded_vector<double,2> VectorND;

		/**
		 * @brief constructor
		 */
		ClarkHullND():
		    pdim(2),
	        vnum(-1),
	        ss(2000)
	    {};

		/**
		 * @brief destructor
		 */
		virtual ~ClarkHullND(){};

		//! @copydoc ConvexHull3D::rebuild
		void rebuild(const std::vector<VectorND>& vertices);

		//! @copydoc ConvexHull3D::isInside
		bool isInside(const VectorND& vertex);

		//! @copydoc ConvexHull3D::getMinDistOutside
		double getMinDistOutside(const VectorND& vertex);

		//! @copydoc ConvexHull3D::getMinDistInside
		double getMinDistInside(const VectorND& vertex);

	public:



    };
	//! @}
}
}
#endif /* GIFTWRAPHULL_HPP_ */
