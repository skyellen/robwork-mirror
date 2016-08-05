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

#ifndef RW_GEOMETRY_QHULL3D_HPP_
#define RW_GEOMETRY_QHULL3D_HPP_

#include <vector>

#include "ConvexHull3D.hpp"

namespace rw {
namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file QHull3D.hpp
     */

    /**
	 * @brief calculates the convex hull of a set of 3d points.
	 *
	 * The GiftWrap convex hull algorithm is used, hence the
	 * class name.
	 *
	 * @note It is important that there are not multiple vertices at the same coordinates.
	 * Filter these away before using this convex hull calculation.
	 */
	class QHull3D: public ConvexHull3D {
	public:
		//! smart pointer type of this class
		typedef rw::common::Ptr<QHull3D> Ptr;

	    /**
		 * @brief constructor
		 */
		QHull3D(){};

		/**
		 * @brief destructor
		 */
		virtual ~QHull3D(){};

		//! @copydoc ConvexHull3D::rebuild
		void rebuild(const std::vector<rw::math::Vector3D<> >& vertices);

		//! @copydoc ConvexHull3D::isInside
        bool isInside(const rw::math::Vector3D<>& vertex);

        //! @copydoc ConvexHull3D::getMinDistOutside
        double getMinDistOutside(const rw::math::Vector3D<>& vertex);

        //! @copydoc ConvexHull3D::getMinDistInside
        double getMinDistInside(const rw::math::Vector3D<>& vertex);

        //! @copydoc ConvexHull3D::toTriMesh
        rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<double> >::Ptr toTriMesh();

		/**
		 * @brief Returns the vertices defining the convex hull
		 */
        const std::vector<rw::math::Vector3D<> >& getVertices() const 
		{
			return _hullVertices;
		};

		/**
		 * @brief Returns the normals of the planes defining the convex hull
		 */
		const std::vector<rw::math::Vector3D<> >& getPlaneNormals() const 
		{
			return _faceNormals;
		}

		/**
		 * @brief Returns the offsets of the planes defining the convex hull
		 */
		const std::vector<double>& getPlaneOffsets() const {
			return _faceOffsets;
		}
	private:
        std::vector<rw::math::Vector3D<> > _hullVertices, _faceNormals;
        std::vector<double> _faceOffsets;
        std::vector<int> _vertiIdxs, _faceIdxs;

	};
	//! @}
}
}
#endif /* GIFTWRAPHULL_HPP_ */
