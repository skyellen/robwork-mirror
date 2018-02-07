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

#ifndef RW_GEOMETRY_POLYGON_HPP_
#define RW_GEOMETRY_POLYGON_HPP_

/** 
 * @file Polygon.hpp
 *
 * \copydoc rw::geometry::Polygon
 */

#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

/**
 * @brief indexed polygon class that saves \b N indices to the \b N vertices of the polygon
 */
template <class T=rw::math::Vector3D<> >
class Polygon {
public:

	//! @brief Smart pointer to Polygon
	typedef rw::common::Ptr<Polygon<T> > Ptr;

	//! @brief value type of the index pointer
	typedef T value_type;

	/**
	 * @brief Adds a vertex to the polygon
	 *
	 * The point will be added to the end of the list of points
	 * @param p [in] The point to add
	 */
	void addVertex(const T& p) {
		_vertices.push_back(p);
	}
	
	/**
	 * @brief Removes vertex from the polygon
	 *
	 * @param idx [in] Index of the vertex to remove
	 */
	void removeVertex(size_t idx) {
		RW_ASSERT_MSG(idx<_vertices.size(), "The requested index "<<idx<<" is not less than the number of items: " << _vertices.size());
		_vertices.erase(_vertices.begin() + idx);		
	}
	
	/**
	 * @brief returns the index of vertex i of the triangle
	 */
	T& getVertex(size_t idx) {
		RW_ASSERT_MSG(idx<_vertices.size(), "The requested index "<<idx<<" is not less than the number of items: " << _vertices.size());	
		return _vertices[idx];
	}

	/**
	 * @brief returns the index of vertex i of the triangle
	 */
	const T& getVertex(size_t idx) const {
		RW_ASSERT_MSG(idx<_vertices.size(), "The requested index "<<idx<<" is not less than the number of items: " << _vertices.size());	
		return _vertices[idx];		
	}

	/**
	 * @brief get vertex at index i
	 */
	T& operator[](size_t i) {
		return getVertex(i);
	}

	/**
	 * @brief get vertex at index i
	 */
	const T& operator[](size_t i) const {
		return getVertex(i);
	}

	/**
	 * @brief Number of vertices of this polygon
	 * @return Number of vertices
	 */
	size_t size() const {
		return _vertices.size(); 
	}

	/**
	 * @brief Computes the center of the polygon as the average of all coordinates
	 * @return Center of the polygon
	 */
	T computeCenter() {
		T sum;
		for (size_t i = 0; i<_vertices.size(); i++) {
			sum += _vertices[i];
		}
		return sum / (double) _vertices.size();
	}

protected:
	/**
	 * @brief Vertices making up the polygon
	 */
	std::vector<T> _vertices;
};

    // @}
} // geometry
} // rw

#endif /*RW_GEOMETRY_POLYGON_HPP_*/
