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


#ifndef RW_GEOMETRY_INDEXEDPOLYGON_HPP_
#define RW_GEOMETRY_INDEXEDPOLYGON_HPP_

#include <rw/common/macros.hpp>
#include <rw/common/types.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief indexed polygon class that saves \b N indices to the \b N vertices of the polygon
	 */
    template <class T=uint16_t>
    class IndexedPolygon {
    public:
        typedef T value_type;

        /**
         * @brief returns the index of vertex i of the triangle
         */
        virtual T& getVertexIdx(size_t i) = 0;

        /**
         * @brief returns the index of vertex i of the triangle
         */
        virtual const T& getVertexIdx(size_t i) const = 0;

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

        virtual size_t size() const = 0;

        //virtual rw::math::Vector3D<T> calcFaceNormal()
    };

	template<class T=uint16_t>
	class IndexedPolygonN : public IndexedPolygon<T> {
	protected:
		boost::numeric::ublas::vector<T> _vertices;

	public:
	    //@brief default constructor

	    IndexedPolygonN(size_t n):
	    	_vertices(n)
	    {};

	    /**
	     * @brief returns the index of vertex i of the triangle
	     */
	    T& getVertexIdx(size_t i) {
	    	RW_ASSERT(i<_vertices.size());
			return _vertices[i];
		}

	    /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertexIdx(size_t i) const  {
        	RW_ASSERT(i<_vertices.size());
            return _vertices[i];
        }

        size_t size() const{ return _vertices.size(); };

	};

	/**
	 * @brief Polygon with N vertices and N normals.
	 */
   template<class T>
    class IndexedPolygonNN : public IndexedPolygon<T> {
    protected:
    	IndexedPolygonN<T> _polyN;
    	boost::numeric::ublas::vector<T> _normals;
    public:
        //@brief default constructor

        IndexedPolygonNN(size_t n):
        	_polyN(n),
        	_normals(n)
        {};

        /**
         * @brief returns the index of vertex i of the triangle
         */
        T& getVertexIdx(size_t i) {
            return _polyN.getVertexIdx(i);
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertexIdx(size_t i) const  {
            return _polyN.getVertexIdx(i);
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        T& getNormalIdx(size_t i) {
            return _normals[i];
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getNormalIdx(size_t i) const  {
            return _normals[i];
        }

    };
    // @}
} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
