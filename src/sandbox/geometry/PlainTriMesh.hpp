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


#ifndef RW_GEOMETRY_PLAINTRIMESH_HPP_
#define RW_GEOMETRY_PLAINTRIMESH_HPP_

#include "TriMesh.hpp"

        template<class Q, class P>
        rw::math::Vector3D<Q> cast2(const rw::math::Vector3D<P>& v)
        {
            return rw::math::Vector3D<Q>(
                static_cast<Q>(v(0)),
                static_cast<Q>(v(1)),
                static_cast<Q>(v(2)));
        }


namespace rw {
namespace geometry {
namespace sandbox {


	/**
	 * @brief
	 */
	template <class TRI>
	class PlainTriMesh: public TriMesh/*<typename TRI::value_type>*/ {
	private:
		std::vector<TRI> _triangles;

	public:

	    typedef typename TRI::value_type value_type;
		/**
		 * @brief constructor
		 */
		PlainTriMesh(int initSize=0):
		    _triangles(initSize)
		{}

		/**
		 * @brief add a triangle to the triangle mesh.
		 */
		void add(const TRI& triangle){
			_triangles.push_back(triangle);
		}

		/**
		 * @brief
		 */
		std::vector<TRI>& getTriangles(){
			return _triangles;
		}

		/**
		 * @brief returns triangle at index i
		 */
		const TRI& operator[](size_t i) const {
			return _triangles[i];
		}

		/**
		* @brief returns triangle at index i
		*/
		TRI& operator[](size_t i) {
		    return _triangles[i];
		}

		void resize(size_t i){
		    _triangles.resize(i);
		}

		// Inherited from TriMesh
		/**
		 * @copydoc TriMesh::getTriangle
		 */
		TriangleN0<double> getTriangle(size_t idx) const {
		    using namespace rw::math;
		    const Triangle<value_type>& triA = _triangles[idx];
		    Vector3D<double> v0 = cast<double>( triA[0] );
		    Vector3D<double> v1 = cast<double>( triA[1] );
		    Vector3D<double> v2 = cast<double>( triA[2] );
			return TriangleN0<double>(v0,v1,v2);
		}

		/**
		 * @copydoc TriMesh::getSize
		 */
		size_t getSize() const {
			return _triangles.size();
		}

		GeometryData::GeometryType getType(){
		    return GeometryData::PlainTriMesh;
		};

	};
}
} // geometry
} // rw

#endif /*TRIMESH_HPP_*/
