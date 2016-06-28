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


#ifndef RW_GEOMETRY_INDEXEDTRIANGLE_HPP_
#define RW_GEOMETRY_INDEXEDTRIANGLE_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/common/types.hpp>
#include <boost/cstdint.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief indexed triangle class that has 3 indices that points to 3
	 * vertices in an array typically used with the IndexedTriMesh  class.
	 * the indice type (size) is templated.
	 */
	template<class T = uint16_t>
	class IndexedTriangle {
	private:
		//! @brief indices to the vertices of the triangle
		T _vertices[3];

	public:
	    //! @brief default constructor
	    IndexedTriangle(){};

	    /**
	     * @brief constructor
	     * @param p1 [in] indice to vertice 1
	     * @param p2 [in] indice to vertice 2
	     * @param p3 [in] indice to vertice 3
	     */
	    IndexedTriangle(T p1, T p2, T p3)
	    {
	    	_vertices[0] = p1;
	    	_vertices[1] = p2;
	    	_vertices[2] = p3;
	    }

	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangle(const IndexedTriangle& f){
	        _vertices[0] = f[0];
	        _vertices[1] = f[1];
	        _vertices[2] = f[2];
	    };

	    /**
	     * @brief returns the index of vertex i of the triangle
	     */
	    T& getVertexIdx(size_t i) {
			return _vertices[i];
		}

	    /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertexIdx(size_t i) const  {
            return _vertices[i];
        }

		/**
		 * @brief tests wheather the point x is inside the triangle
		 */
        template<class R>
		bool isInside(const rw::math::Vector3D<R>& x, const std::vector<rw::math::Vector3D<R> >& verts){
			using namespace rw::math;
			// calc vectors
			Vector3D<R> v0 = verts[_vertices[2]] - verts[_vertices[0]];
			Vector3D<R> v1 = verts[_vertices[1]] - verts[_vertices[0]];
			Vector3D<R> v2 = x - verts[_vertices[0]];
			// calc dot products
			R dot00 = dot(v0, v0);
			R dot01 = dot(v0, v1);
			R dot02 = dot(v0, v2);
			R dot11 = dot(v1, v1);
			R dot12 = dot(v1, v2);
			// calc barycentric coordinates
			R invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
			R u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			R v = (dot00 * dot12 - dot01 * dot02) * invDenom;

			// Check if point is in triangle
			return (u > 0) && (v > 0) && (u + v < 1);
		}


        /**
         * @brief get vertex at index i
         */
        T& operator[](size_t i){
            return getVertexIdx(i);
        }

        /**
         * @brief get vertex at index i
         */
        const T& operator[](size_t i) const {
            return getVertexIdx(i);
        }

	};

	/**
	 * @brief indexed triangle class with an additional index for an face normal
	 */
    template<class T>
    class IndexedTriangleN1 {
    private:
        IndexedTriangle<T> _triN0;
        T _normalIdx;
    public:
        //@brief default constructor

        /**
         * @brief constructor
         */
        IndexedTriangleN1(){};

        /**
         * @brief constructor
         * @param p1 [in] vertice index 1
         * @param p2 [in] vertice index 2
         * @param p3 [in] vertice index 3
         * @param n [in] normal index
         */
        IndexedTriangleN1(T p1, T p2, T p3, T n):
            _triN0(p1,p2,p3),_normalIdx(n)
        {
        };

        /**
         * @brief copy constructor
         *
         * @param f [in] - The face that is to be copied.
         */
        IndexedTriangleN1(const IndexedTriangleN1& f):
            _triN0(f._triN0),
            _normalIdx(f.getNormalIdx())
        {
        };

        /**
         * @brief returns the index of vertex i of the triangle
         */
        T& getVertexIdx(size_t i) {
            return _triN0.getVertexIdx(i);
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertexIdx(size_t i) const  {
            return _triN0.getVertexIdx(i);
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        T& getNormalIdx() {
            return _normalIdx;
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getNormalIdx() const  {
            return _normalIdx;
        }


        /**
         * @brief tests wheather the point x is inside the triangle
         */
        template<class R>
        bool isInside(const rw::math::Vector3D<R>& x, const std::vector<rw::math::Vector3D<R> >& verts){
            return _triN0.template isInside<R>(x,verts);
        }

        /**
         * @brief get vertex at index i
         */
        T& operator[](size_t i){
            return getVertexIdx(i);
        }

        /**
         * @brief get vertex at index i
         */
        const T& operator[](size_t i) const {
            return getVertexIdx(i);
        }

    };

	/**
	 * @brief indexed triangle class with an additional index for 3 normals one for each vertice in the triangle
	 */
    template<class T>
     class IndexedTriangleN3  {
     private:
         IndexedTriangle<T> _triN0;
         T _normals[3];
     public:
         //@brief default constructor
         IndexedTriangleN3(){};

         /**
          * @brief constructor
          * @param p1 [in] vertice index 1
          * @param p2 [in] vertice index 2
          * @param p3 [in] vertice index 3
          * @param n1 [in] normal index for vertice 1
          * @param n2 [in] normal index for vertice 2
          * @param n3 [in] normal index for vertice 3
          */
         IndexedTriangleN3(T p1, T p2, T p3,
                           T n1, T n2, T n3):
             _triN0(p1,p2,p3)
         {
             _normals[0] = n1;
             _normals[1] = n2;
             _normals[2] = n3;
         };

         /**
          * @brief copy constructor
          *
          * @param f [in] - The face that is to be copied.
          */
         IndexedTriangleN3(const IndexedTriangleN3& f):
             _triN0(f._triN0)
         {
             _normals[0] = f._normals[0];
             _normals[1] = f._normals[0];
             _normals[2] = f._normals[0];
         };

         /**
          * @brief returns the index of vertex i of the triangle
          */
         T& getVertexIdx(size_t i) {
             return _triN0.getVertexIdx(i);
         }

         /**
          * @brief returns the index of vertex i of the triangle
          */
         const T& getVertexIdx(size_t i) const  {
             return _triN0.getVertexIdx(i);
         }

         /**
          * @brief get the index of vertice normal \b i.
          * @param i [in] vertice normal
          * @return
          */
         T& getNormalIdx(size_t i) {
             return _triN0.getVertexIdx(i);
         }

         /**
          * @brief returns the index of vertex i of the triangle
          */
         const T& getNormalIdx(size_t i) const  {
             return _triN0.getVertexIdx(i);
         }

         /**
          * @brief tests wheather the point x is inside the triangle
          */
         template<class R>
         bool isInside(const rw::math::Vector3D<R>& x, const std::vector<rw::math::Vector3D<R> >& verts){
             return _triN0.template isInside<R>(x,verts);
         }

         /**
          * @brief get vertex at index i
          */
         T& operator[](size_t i){
             return getVertexIdx(i);
         }

         /**
          * @brief get vertex at index i
          */
         const T& operator[](size_t i) const {
             return getVertexIdx(i);
         }

     };
    // @}
} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
