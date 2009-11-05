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

#include "GeometryData.hpp"
#include "Triangle.hpp"

namespace rw {
namespace geometry {
namespace sandbox {

	/**
	 * @brief interface of a triangle mesh. The interface defines a way to get
	 * triangles from a triangle array/mesh.
	 */
	//template <class T=double>
	class TriMesh: public GeometryData {
	public:
		//typedef T value_type;

		/**
		 * @brief destructor
		 */
		virtual ~TriMesh(){};

		/**
		 * @brief gets the triangle at index idx.
		 */
		virtual TriangleN0<double> getTriangle(size_t idx) const = 0;

		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t getSize() const = 0;

		virtual size_t size() const{
		    return getSize();
		}
	};

}
} // geometry
} // rw


#endif /*TRIMESH_HPP_*/
