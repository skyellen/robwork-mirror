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


#ifndef RW_GEOMETRY_INDEXEDTRIMESH_HPP_
#define RW_GEOMETRY_INDEXEDTRIMESH_HPP_

#include <rw/math/Vector3D.hpp>
#include "TriMesh.hpp"
#include "Triangle.hpp"
#include "IndexedTriangle.hpp"



namespace rw {
namespace geometry {
	typedef enum {V1=0,V2,V3} VertexIdx;

	/**
	 * @brief Interface for indexed triangle meshes.
	 *
	 * An indexed triangle mesh has a list of vertices and a list of
	 * indicies. The indicies is used as reference into the vertex list.
	 * Indicies are grouped into threes such that one triangle is defined
	 * by three following indicies.
	 *
	 * In the IndexedTriMesh classes the indice list is hidden under a list
	 * of IndexedTriangle.
	 */
    template <class T = double>
    class IndexedTriMesh: public TriMesh {
    public:
    	//! the basic value type of this mesh
    	typedef T value_type;
    	typedef std::vector<rw::math::Vector3D<T> > VertexArray;

    protected:

    	IndexedTriMesh(VertexArray *vertices, VertexArray *normals,
    				uint8_t triStride, uint8_t idxsize):
    					_vertices(vertices),
    					_normals(normals),
    					_triIdxArr(NULL),
    					_stride(triStride),
    					_idxsize(idxsize)
		{
    		_mask = 0;
    		for(uint8_t i=0;i<idxsize;i++)
    			_mask |= 0xFF<<(i*8);
    		//std::cout << "MASK: " << std::hex << _mask << std::endl;
		}

    public:

    	//! @brief get array of normals
        const std::vector<rw::math::Vector3D<T> >& getNormals() const{return *_normals;}

        //! @brief get array of normals
        std::vector<rw::math::Vector3D<T> >& getNormals(){return *_normals;};

        //! @brief get array of vertices
        const std::vector<rw::math::Vector3D<T> >& getVertices() const{
        	RW_ASSERT(_vertices);
        	return *_vertices;
        };

        //! @brief get array of vertices
        std::vector<rw::math::Vector3D<T> >& getVertices(){
			RW_ASSERT(_vertices);
        	return *_vertices;
        };

        //! @brief the stride of a triangle
        uint8_t getTriangleStride(){ return _stride;};

        //! @brief the size of the index type used, typically uint16_t or uint32_t
        uint8_t getIndexSize(){ return _idxsize;};

        //! @brief pointer to the start of the index array
        uint8_t* getIndexPtr(){ return _triIdxArr;};

        //! @brief get number of triangles
        size_t size(){ return getNrTris();};

        //! @brief return a vertice indexed into the vertice list
        const rw::math::Vector3D<T>& getVertex(size_t i) const{ return _vertices->at(i);};

        //! @brief return a vertice indexed into the vertice list
        rw::math::Vector3D<T>& getVertex(size_t i){ return _vertices->at(i);};

		/**
		 * @brief get vertex \b vidx of triangle at index \b i.
         * @param i [in] the index of the triangle
         * @param vidx [in] the index of the triangle vertex
		 */
        const rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx) const{
        	const uint32_t idx = _stride*i+vidx*_idxsize; // this is the unmasked idx
        	const uint32_t uidx =  *((uint32_t*)&(_triIdxArr[ idx ]))&_mask; // now we mask the idx
        	return (*_vertices)[ uidx ];
        }

		/**
		 * @brief get vertex \b vidx of triangle at index \b i.
         * @param i [in] the index of the triangle
         * @param vidx [in] the index of the triangle vertex
		 */
        rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx){
        	const uint32_t idx = _stride*i+vidx*_idxsize; // this is the unmasked idx
        	const uint32_t uidx =  *((uint32_t*)&(_triIdxArr[ idx ]))&_mask; // now we mask the idx
        	return (*_vertices)[ uidx ];
        }

        /**
         * @brief get the indexed triangle at index \b i
         * @param i [in] the index of the triangle
         * @return an indexed triangle
         */
        IndexedTriangle<uint32_t> getIndexedTriangle(size_t i) const {
        	const uint32_t idx = _stride*i; // this is the unmasked idx
        	const uint32_t v1 = *((uint32_t*)&(_triIdxArr[ idx ]));
        	const uint32_t v2 = *((uint32_t*)&(_triIdxArr[ idx+_idxsize ]));
        	const uint32_t v3 = *((uint32_t*)&(_triIdxArr[ idx+2*_idxsize ]));
        	if((v1&_mask)>getNrTris()){
        		std::cout << "index: " << idx << ", " << i<< std::endl;
        		std::cout << "stride: " << ((int)_stride) << std::endl;
        		std::cout << "mask: " << _mask << std::endl;
        		std::cout << v1 << ", " << v2 << ", " << v3 << std::endl;
        		std::cout << (v1&_mask) << ", " << (v2&_mask) << ", " << (v3&_mask) << std::endl;
        	}
        	return IndexedTriangle<uint32_t>(v1&_mask,v2&_mask,v3&_mask);
        }

        /**
         * @brief The number of triangles in the mesh
         * @return nr of triangles
         */
        virtual int getNrTris() const = 0 ;

        /**
         * @copydoc GeomtryData::getType
         */
        GeometryData::GeometryType getType() const{
            return GeometryData::IdxTriMesh;
        };

    protected:
		VertexArray *_vertices;
		VertexArray *_normals;
		uint8_t *_triIdxArr;
		void setTriArray(uint8_t *triarray){ _triIdxArr = triarray;};
    private:
		const uint8_t _stride, _idxsize;
		uint32_t _mask;
    };

	/**
	 *
	 * @brief an Indexed Triangle mesh
	 *
	 */
	template <class T=double, class S=uint16_t>
	class IndexedTriMeshN0: public IndexedTriMesh<T> {
	public:
	    typedef T value_type;
	    typedef S index_type;
	    typedef IndexedTriangle<S> tri_type;
	    typedef IndexedTriangle<S> TRI;
		typedef std::vector<TRI> TriangleArray;
		typedef std::vector<rw::math::Vector3D<T> > VertexArray;

	private:
	    //typedef value_type T;
		TriangleArray *_triangles;

	public:

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0():
			IndexedTriMesh<T>(
					new VertexArray(), new VertexArray(),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( new TriangleArray() )
		{
			//this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in]
		 */
		IndexedTriMeshN0(std::vector<rw::math::Vector3D<T> > *vertices):
			IndexedTriMesh<T>(
					vertices,
					new VertexArray(),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( new TriangleArray() )
		{
			//this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};


		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in]
		 */
		IndexedTriMeshN0(
				std::vector<rw::math::Vector3D<T> > *vertices,
				std::vector<rw::math::Vector3D<T> > *normals):
			IndexedTriMesh<T>(
					vertices,
					normals,
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( new TriangleArray() )
		{
			//this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0(
				std::vector<rw::math::Vector3D<T> > *vertices,
				std::vector<rw::math::Vector3D<T> > *normals,
				TriangleArray *triangles):
			IndexedTriMesh<T>(
					vertices,
					normals,
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( triangles )
		{
			RW_ASSERT( _triangles );
			if(triangles!=NULL && triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		IndexedTriMeshN0(VertexArray *vertices, TriangleArray *triangles):
			IndexedTriMesh<T>(
					vertices,
					new VertexArray(),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( triangles )
		{
			RW_ASSERT( _triangles );
			if(triangles!=NULL && triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief destructor
		 */
		virtual ~IndexedTriMeshN0(){
			if(_triangles)
				delete _triangles;
		};

		/**
		 * @brief add indexed triangle to the triangle mesh.
		 */
		void add(const TRI& triangle){
			_triangles->push_back(triangle);
			this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		}

		/**
		 * @brief get vertex i of triangle at index triIdx.
		 * @param i [in] should be in interval [0;2]
		 * @param triIdx [in] index of triangle in the triangle mesh
		 */
		const rw::math::Vector3D<T>& getTriVertex(size_t i, size_t triIdx) const {
			const TRI& tri = (*_triangles)[triIdx];
			return (*this->_vertices)[tri.getVertexIdx(i)];
		}

		/**
		 * @brief get vertex i of triangle at index triIdx.
		 * @param i [in] should be in interval [0;2]
		 * @param triIdx [in] index of triangle in the triangle mesh
		 */
		rw::math::Vector3D<T>& getTriVertex(size_t i, size_t triIdx) {
			const TRI& tri = (*_triangles)[triIdx];
			return (*this->_vertices)[tri.getVertexIdx(i)];
		}

		/**
		 * @brief get triangle list
		 * @return vector of triangles
		 */
		TriangleArray& getTriangles(){
			return *_triangles;
		}

		/**
		 * @brief get triangle list
		 * @return vector of triangles
		 */
		const TriangleArray& getTriangles() const {
			return *_triangles;
		}

        /**
         * @brief get vertex at index i
         */
        TRI& operator[](int i){
            return (*_triangles)[i];
        }

        const TRI& operator[](int i) const {
            return (*_triangles)[i];
        }

		T calcFaceArea(size_t triIdx){
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return ( cross( v0-v1 , v0-v2 ) ).norm2()/2;
		}

		rw::math::Vector3D<T> calcFaceCentroid(size_t triIdx){
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return (v0+v1+v2)/3;
		}

		rw::math::Vector3D<T> calcFaceNormal(size_t triIdx) const {
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			Vector3D<T> n = cross(v1-v0,v2-v0);
	        return normalize(n);
		}

		// inherited from IndexedTriMesh
		//! @copydoc IndexedTriMesh::getNrTris
        int getNrTris() const{ return _triangles->size(); };

		// Inherited from TriMesh
        //! @copydoc IndexedTriMesh::getTriangle
		Triangle<double> getTriangle(size_t idx) const {
			using namespace rw::math;
			const TRI& tri = (*_triangles)[idx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return Triangle<double>(cast<double>(v0),cast<double>(v1),cast<double>(v2));
		}

		//! @copydoc TriMesh::getSize
		size_t getSize() const {return _triangles->size(); }

		//! @copydoc TriMesh::clone
		TriMeshPtr clone() const {
			return rw::common::ownedPtr( new IndexedTriMeshN0(*this) );
		}

	};
}
} // geometry


#endif /*INDEXEDTRIMESH_HPP_*/
