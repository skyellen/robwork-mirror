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


#ifndef RW_GEOMETRY_TRIMESH_HPP_
#define RW_GEOMETRY_TRIMESH_HPP_

#include <rw/common/Ptr.hpp>

#include "GeometryData.hpp"
#include "Triangle.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief interface of a triangle mesh. The interface defines a way to get
	 * triangles from a triangle array/mesh.
	 */
	class TriMesh: public GeometryData {
	public:
		/**
		 * @brief destructor
		 */
		virtual ~TriMesh(){};

		/**
		 * @brief gets the triangle at index idx.
		 */
		virtual Triangle<> getTriangle(size_t idx) const = 0;

		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t getSize() const = 0;

		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t size() const = 0;

		/**
		 * @brief make a clone of this triangle mesh
		 * @return clone of this trimesh
		 */
		virtual rw::common::Ptr<TriMesh> clone() const = 0;

		//! @copydoc GeometryData::getTriMesh
		rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

		//! @copydoc getTriMesh
		rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;
	};

	//! @brief Ptr to TriMesh
	typedef rw::common::Ptr<TriMesh> TriMeshPtr;
	//! @}
} // geometry
} // rw



#endif /*TRIMESH_HPP_*/
