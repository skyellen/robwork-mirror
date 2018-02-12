/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_INDEXEDQUADRATICFACEARRAY_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_INDEXEDQUADRATICFACEARRAY_HPP_

/**
 * @file IndexedQuadraticFaceArray.hpp
 *
 * \copydoc rw::geometry::IndexedQuadraticFaceArray
 */

#include "QuadraticShell.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief Proxy class for a QuadraticShell, that allows sorting of faces without modifying the original object.
 */
class IndexedQuadraticFaceArray: public QuadraticShell {
public:
    //! @brief Smart pointer type to IndexedQuadraticFaceArray
    typedef rw::common::Ptr<IndexedQuadraticFaceArray> Ptr;

	//! @brief Smart pointer type for a const IndexedQuadraticFaceArray.
	typedef rw::common::Ptr<const IndexedQuadraticFaceArray> CPtr;

	//! @brief Structure that holds information for each face.
	struct IndexedFace {
		//! @brief Constructor.
		IndexedFace(): originalID(0), lower(0), upper(0), center(0) {}

		//! @brief Index of the original face.
		std::size_t originalID;

		//! @brief Lower limit used for sorting.
		double lower;
		//! @brief Upper limit used for sorting.
		double upper;
		//! @brief Center of limits used for sorting.
		double center;
	};

	/**
	 * @brief Construct new indexed face array.
	 * @param quadric [in] the underlying Shell.
	 */
	IndexedQuadraticFaceArray(rw::common::Ptr<const QuadraticShell> quadric);

	/**
	 * @brief Construct new indexed face array.
	 * @param quadric [in] the underlying Shell.
	 * @param faces [in] the faces to include in the proxy.
	 * @param first [in] skip the \b first \b faces.
	 * @param last [in] last index of \b faces to include.
	 */
	IndexedQuadraticFaceArray(rw::common::Ptr<const QuadraticShell> quadric, const std::vector<IndexedFace>& faces, std::size_t first, std::size_t last);

	//! @brief Destructor.
	virtual ~IndexedQuadraticFaceArray();

	//! @copydoc QuadraticShell::isConvex
	virtual bool isConvex();

	//! @copydoc QuadraticShell::size
	virtual std::size_t size() const;

	//! @copydoc QuadraticShell::getFace(std::size_t) const
	virtual rw::common::Ptr<const QuadraticFace> getFace(std::size_t idx) const;

	//! @copydoc QuadraticShell::getFace(std::size_t,QuadraticFace&) const
	virtual void getFace(std::size_t idx, QuadraticFace& dst) const;

	/**
	 * @brief Get the indexed face.
	 * @param idx [in] index of indexed face.
	 * @return the indexed face.
	 */
	IndexedFace getIndexedFace(std::size_t idx) const;

	/**
	 * @brief Get the indexed face.
	 * @param idx [in] index of indexed face.
	 * @param dst [out] existing object to copy data into.
	 */
	void getIndexedFace(std::size_t idx, IndexedFace& dst) const;

	/**
	 * @brief Sort the faces according to their extent in the direction along \b axis.
	 * @param axis [in] axis to sort.
	 * @param t3d [in] transform giving the position and axis directions.
	 */
	void sortAxis(int axis, const rw::math::Transform3D<>& t3d);

	/**
	 * @brief Take out a subrange of faces.
	 * @param first [in] first index.
	 * @param last [in] last index.
	 * @return a new indexed face array.
	 */
	IndexedQuadraticFaceArray getSubRange(std::size_t first, std::size_t last) const;

	/**
	 * @brief Get the original face index.
	 * @param idx [in] the indexed face index.
	 * @return the original index.
	 */
	inline std::size_t getGlobalIndex(std::size_t idx) const { return _faces[_first+idx].originalID; }

private:
	struct CenterSort;

	const rw::common::Ptr<const QuadraticShell> _quadric;
	std::vector<IndexedFace> _faces;
	std::size_t _first;
	std::size_t _last;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_INDEXEDQUADRATICFACEARRAY_HPP_ */
