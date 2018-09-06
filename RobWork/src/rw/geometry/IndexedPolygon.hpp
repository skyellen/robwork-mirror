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
//#include <rw/math/Vector3D.hpp>



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
	
		//! @brief Smart pointer to IndexedPolygonN
		typedef rw::common::Ptr<IndexedPolygon<T> > Ptr;
	
        //! @brief value type of the index pointer
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

        /**
         * @brief Number of vertices of this polygon
         * @return Number of vertices
         */
        virtual size_t size() const = 0;

        //virtual rw::math::Vector3D<T> calcFaceNormal()
    };

    /**
     * @brief Polygon with N vertice indices and 0 normals
     */
	template<class T=uint16_t>
	class IndexedPolygonN : public IndexedPolygon<T> {
	protected:
		//! @brief The vertices of the polygon.
		std::vector<T> _vertices;

	public:
		//! @brief Smart pointer to IndexedPolygonN
		typedef rw::common::Ptr<IndexedPolygonN<T> > Ptr;

	    //! @brief Constructs IndexedPolygon with space for n vertices
		IndexedPolygonN()
		{}

	    //! @brief Constructs IndexedPolygon with space for n vertices
	    IndexedPolygonN(size_t n):
	    	_vertices(n)
	    {}

		/** 
		 * @brief Constructs IndexedPolygonN with the vertices specified
		 */
		IndexedPolygonN(const std::vector<T>& vertices):
			_vertices(vertices)
		{
		}

	    /**
	     * @brief returns the index of vertex i of the triangle
	     */
	    T& getVertexIdx(size_t i) {
	        RW_ASSERT_MSG(i<_vertices.size(), i<<"<"<<_vertices.size());
			return _vertices[i];
		}

	    /**
         * @brief returns the index of vertex i of the triangle
         */
        const T& getVertexIdx(size_t i) const  {
        	RW_ASSERT(i<_vertices.size());
            return _vertices[i];
        }

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
		 * @param i [in] Index of the point to remove
		 */
		void removeVertexIdx(size_t i) {
                    RW_ASSERT_MSG(i<_vertices.size(), "The requested index "<<i<<" is not less than the number of items: " << _vertices.size());
			_vertices.erase(_vertices.begin() + i);		
		}

		/**
		 * @copydoc IndexedPolygon::size
		 */
        size_t size() const{ return _vertices.size(); };

	};

	/**
	 * @brief Polygon with N vertices and N normals.
	 */
   template<class T>
    class IndexedPolygonNN : public IndexedPolygon<T> {
    private:
    	IndexedPolygonN<T> _polyN;
    	//boost::numeric::ublas::vector<T> _normals;
		std::vector<T> _normals;
    public:
		//! @brief Smart pointer to IndexedPolygonNN
		typedef rw::common::Ptr<IndexedPolygonNN<T> > Ptr;

        //! @brief Construct IndexPolygonNN with space for n vertices and normals
        IndexedPolygonNN(size_t n):
        	_polyN(n),
        	_normals(n)
        {}

		/** 
		 * @brief Constructs IndexedPolygonN-n with the vertices and normals specified
		 *
		 * In case the number of vertices and normals does not match the method either asserts or throws an exception.
		 *
		 * @param vertices [in] The vertices to be part of the polygon
		 * @param normals [in] The normals associated to the vertices
		 */
		IndexedPolygonNN(const std::vector<T>& vertices, const std::vector<T>& normals):
			_polyN(vertices),
			_normals(normals)
		{
			RW_ASSERT_MSG(vertices.size() == _normals.size(), "The number of vertices and normals does not match.");
			if (vertices.size() != normals.size()) 
				RW_THROW("The number of vertices (" << vertices.size() << ") and normals (" << normals.size() << ") does not match.");
		}



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

		/**
		 * @brief Adds a vertex to the polygon
		 *
		 * The point will be added to the end of the list of points
		 * @param p [in] The point to add
		 * @param n [in] Normal associated to the point
		 */
		void addVertex(const T& p, const T& n) {
                    _polyN.addVertex(p);
                    _normals.push_back(n);
		}

		/**
		 * @brief Removes vertex from the polygon
		 *
		 * @param i [in] Index of the point to remove
		 */
		void removeVertexIdx(size_t i) {
                    RW_ASSERT_MSG(i < _polyN.size() && i < _normals.size(), "The requested index " << i << " is not less than the number of items: " << _polyN.size() << " and " << _normals.size());
			_polyN.removeVertexIdx(i);		
			_normals.erase(_normals.begin() + i);
		}

    };
    // @}
} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
