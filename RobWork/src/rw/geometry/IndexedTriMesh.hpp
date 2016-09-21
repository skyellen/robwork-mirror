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
#include <rw/common/Ptr.hpp>
#include "TriMesh.hpp"
#include "Triangle.hpp"
#include "IndexedTriangle.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	//! vertice indexes of triangle
	typedef enum {
		V1=0,//!< The first vertex.
		V2,  //!< The second vertex.
		V3   //!< The third vertex.
	} VertexIdx;

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
    	//! the basic value type of this mesh (for instance double or float)
    	typedef T value_type;
    	//! the vertex array type
    	typedef std::vector<rw::math::Vector3D<T> > VertexArray;
    	//! the smart pointer type of this triangle mesh
    	typedef rw::common::Ptr<IndexedTriMesh<T> > Ptr;

    protected:

    	/**
    	 * @brief generic constructor
    	 * @param vertices [in] pointer to vertice array (ownership taken)
    	 * @param normals [in] pointer to normal array (NULL if not used) (ownership taken)
    	 * @param triStride [in] the stride of the indice array (set using setTriArray).
    	 * @param idxsize [in] the size in bytes of an indice in the indice array
    	 */
    	IndexedTriMesh(rw::common::Ptr<VertexArray> vertices, rw::common::Ptr<VertexArray> normals,
    				uint8_t triStride, uint8_t idxsize):
    					_vertices(vertices.get()),
    					_vertPtr(vertices),
    					_normals(normals.get()),
    					_normPtr(normals),
    					_triIdxArr(NULL),
    					_stride(triStride),
    					_idxsize(idxsize)
		{
		}

    	//! @brief Destructor.
    	virtual ~IndexedTriMesh(){
     	}

    public:
    	/**
    	 * @brief Get vector of normals.
    	 * @return reference to vector.
    	 */
        const std::vector<rw::math::Vector3D<T> >& getNormals() const{
        	RW_ASSERT(_normals);
        	return *_normals;
        }

    	//! @copydoc IndexedTriMesh::getNormals
        std::vector<rw::math::Vector3D<T> >& getNormals(){
        	RW_ASSERT(_normals);
        	return *_normals;
        }

        /**
         * @brief Get vector of vertices.
         * @return reference to vector.
         */
        const std::vector<rw::math::Vector3D<T> >& getVertices() const{
        	RW_ASSERT(_vertices);
        	return *_vertices;
        }

    	//! @copydoc IndexedTriMesh::getVertices
        std::vector<rw::math::Vector3D<T> >& getVertices(){
			RW_ASSERT(_vertices);
        	return *_vertices;
        }

        /**
         * @brief The stride of a triangle.
         * @return the stride.
         */
        uint8_t getTriangleStride() const { return _stride;}

        /**
         * @brief The size of the index type used.
         * @return Typically 2 (uint16_t) or 4 (uint32_t).
         */
        uint8_t getIndexSize() const { return _idxsize;}

        /**
         * @brief Pointer to the start of the index array.
         * @return pointer to byte array.
         */
        uint8_t* getIndexPtr(){ return _triIdxArr;}

        //! @copydoc TriMesh::size
        virtual size_t size() const { return getNrTris();}

        /**
         * @brief Get vertex from vertex list.
         * @param i [in] the vertex id (NOT the triangle id).
         * @return a reference to the vertex.
         */
        const rw::math::Vector3D<T>& getVertex(size_t i) const { return _vertices->at(i);}

        /**
         * @brief Get vertex from vertex list.
         * @param i [in] the vertex id (NOT the triangle id).
         * @return a reference to the vertex.
         */
        rw::math::Vector3D<T>& getVertex(size_t i) { return _vertices->at(i);}

        /**
         * @brief Check if trimesh has vertex normals defined.
         * @return true if vertex normals are defined.
         */
        bool hasVertexNormals() const { return _normals->size() == _vertices->size(); }

        /**
         * @brief Check if trimesh has face normals defined.
         * @return true if face normals are defined.
         */
        bool hasFaceNormals() const { return _normals->size() == getNrTris(); }

        /**
         * @brief Get normal of vertex \b vidx of triangle at index \b i.
         * @param i [in] the index of the triangle.
         * @param vidx [in] the triangle vertex.
         * @return reference to normal vector.
         * @warning Using the virtual function comes with a performance penalty.
         * If possible, use functions implemented on subtypes directly.
         */
        virtual const rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx) const = 0;

    	//! @copydoc IndexedTriMesh::getVertexNormal
        virtual rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx) = 0;

		/**
		 * @brief get vertex \b vidx of triangle at index \b i.
         * @param i [in] the index of the triangle
         * @param vidx [in] the index of the triangle vertex
         * @return reference to vertex.
         * @warning Using the virtual function comes with a performance penalty.
         * If possible, use functions implemented on subtypes directly (like IndexedTriMeshN0::getTriVertex).
		 */
        virtual const rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx) const = 0;

    	//! @copydoc IndexedTriMesh::getVertex(size_t,VertexIdx) const
        virtual rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx) = 0;

        /**
         * @brief get the indexed triangle at index \b i
         * @param i [in] the index of the triangle
         * @return an indexed triangle
         * @warning Using the virtual function comes with a performance penalty.
         * If possible, use functions implemented on subtypes directly.
         */
        virtual IndexedTriangle<uint32_t> getIndexedTriangle(size_t i) const = 0;

        /**
         * @brief The number of triangles in the mesh
         * @return nr of triangles
         */
        virtual int getNrTris() const = 0;

        //! @copydoc GeometryData::getType
        GeometryData::GeometryType getType() const {
            return GeometryData::IdxTriMesh;
        }

        /**
         * @brief Get pointer to first element of index array.
         * @return pointer to first element.
         */
        void* getIndices() {return (void*)_triIdxArr;}

    private:
        IndexedTriMesh(IndexedTriMesh<T>& mesh): _vertices(NULL), _normals(NULL), _triIdxArr(NULL), _stride(0), _idxsize(0) {} // this is illegal

    protected:
        //! @brief pointer to vertice array
		VertexArray *_vertices;
		//! @brief Smart pointer to maintain ownership of vertice array.
		rw::common::Ptr<VertexArray> _vertPtr;
		//! @brief pointer to normal array
		VertexArray *_normals;
		//! @brief Smart pointer to maintain ownership of normals array.
		rw::common::Ptr<VertexArray> _normPtr;
		//! @brief pointer to indice array
		uint8_t *_triIdxArr;
		//! @brief The stride.
		const uint8_t _stride;
		//! @brief Size of index.
		const uint8_t _idxsize;

		/**
		 * @brief Sets the indice array.
		 * @param triarray [in] the array.
		 */
		void setTriArray(uint8_t *triarray){ _triIdxArr = triarray;}
    };

    //! @brief IndexedTriMesh using valuetype double
    typedef IndexedTriMesh<> IndexedTriMeshD;

	//! @brief IndexedTriMesh using valuetype float
    typedef IndexedTriMesh<float> IndexedTriMeshF;


	//! @brief smart pointer type of IndexedTriMeshD
    typedef rw::common::Ptr<IndexedTriMesh<> > IndexedTriMeshDPtr;
    //! @brief smart pointer type of IndexedTriMeshF
    typedef rw::common::Ptr<IndexedTriMesh<float> > IndexedTriMeshFPtr;

	/**
	 * @brief an Indexed Triangle mesh with zero normals
	 */
	template <class T=double, class S=uint16_t>
	class IndexedTriMeshN0: public IndexedTriMesh<T> {
	public:
	    //! @brief The type of indices.
	    typedef S index_type;
	    //! @brief The type of the triangles of the mesh.
	    typedef IndexedTriangle<S> tri_type;
	    //! @brief The vector of triangles.
		typedef std::vector<tri_type> TriangleArray;
		//! @copydoc IndexedTriMesh::VertexArray
		typedef typename IndexedTriMesh<T>::VertexArray VertexArray;
		//! @brief Smart pointer type of this class.
		typedef rw::common::Ptr<IndexedTriMeshN0<T, S> > Ptr;

	private:
		rw::common::Ptr<TriangleArray> _triangles;

	public:
		//! @brief Constructor
		IndexedTriMeshN0():
			IndexedTriMesh<T>(
			        rw::common::ownedPtr( new VertexArray() ),
			        rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(tri_type),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
		{
		}

		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in] pointer to vector of vertices.
		 */
		IndexedTriMeshN0(VertexArray* vertices):
			IndexedTriMesh<T>(
					vertices,
					rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(tri_type),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
		{
		}

		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in] pointer to vector of vertices.
		 * @param normals [in] pointer to vector of normals.
		 */
		IndexedTriMeshN0(
				rw::common::Ptr<VertexArray> vertices,
				rw::common::Ptr<VertexArray> normals):
			IndexedTriMesh<T>(
					vertices,
					normals,
					(uint8_t)sizeof(tri_type),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
		{
		}

		/**
		 * @brief constructor
		 * @param vertices [in] pointer to vector of vertices.
		 * @param normals [in] pointer to vector of normals (expects same length as either \b vertices or \b triangles).
		 * @param triangles [in] pointer to array of triangles.
		 */
		IndexedTriMeshN0(
				rw::common::Ptr<VertexArray> vertices,
				rw::common::Ptr<VertexArray> normals,
				rw::common::Ptr<TriangleArray> triangles):
			IndexedTriMesh<T>(
					vertices,
					normals,
					(uint8_t)sizeof(tri_type),
					(uint8_t)sizeof(S)),
			_triangles( triangles )
		{
			RW_ASSERT( _triangles );
			if(triangles!=NULL && triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief constructor
		 * @param vertices [in] pointer to vector of vertices.
		 * @param triangles [in] pointer to array of triangles.
		 */
		IndexedTriMeshN0(rw::common::Ptr<VertexArray> vertices, rw::common::Ptr<TriangleArray> triangles):
			IndexedTriMesh<T>(
					vertices,
					rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(tri_type),
					(uint8_t)sizeof(S)),
			_triangles( triangles )
		{
			RW_ASSERT( _triangles );
			if(triangles!=NULL && triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief Make deep copy of mesh.
		 * @param mesh [in] the mesh to copy.
		 */
		IndexedTriMeshN0(const IndexedTriMeshN0& mesh):
		    IndexedTriMesh<T>(
		            rw::common::ownedPtr(new VertexArray( mesh.getVertices() )),
		            rw::common::ownedPtr(new VertexArray( mesh.getNormals() )),
		            (uint8_t)sizeof(tri_type),
		            (uint8_t)sizeof(S)
		    ),
		    _triangles( rw::common::ownedPtr(new TriangleArray(mesh.getTriangles()) ) )
		{
            if(_triangles!=NULL && _triangles->size()>0)
                this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		}

		//! @brief Destructor
		virtual ~IndexedTriMeshN0() {
		}

        //! @copydoc IndexedTriMesh::getVertexNormal
        const rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx) const {
            const uint32_t idx = static_cast<uint32_t>(this->_stride*i+vidx*this->_idxsize); // this is the unmasked idx
            const S uidx =  *((S*)&(this->_triIdxArr[ idx ]));
            return (*this->_normals)[ uidx ];
        }

        //! @copydoc IndexedTriMesh::getVertexNormal
        rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx) {
            const uint32_t idx = static_cast<uint32_t>(this->_stride*i+vidx*this->_idxsize); // this is the unmasked idx
            const S uidx =  *((S*)&(this->_triIdxArr[ idx ]));
            return (*this->_normals)[ uidx ];
        }

        //! @copydoc IndexedTriMesh::getVertex(size_t,VertexIdx)
        const rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx) const{
        	const uint32_t idx = static_cast<uint32_t>(this->_stride*i+vidx*this->_idxsize); // this is the unmasked idx
        	const S uidx =  *((S*)&(this->_triIdxArr[ idx ]));
        	return (*this->_vertices)[ uidx ];
        }

        //! @copydoc IndexedTriMesh::getVertex(size_t,VertexIdx)
        rw::math::Vector3D<T>& getVertex(size_t i, VertexIdx vidx){
        	const uint32_t idx = static_cast<uint32_t>(this->_stride*i+vidx*this->_idxsize); // this is the unmasked idx
        	const S uidx =  *((S*)&(this->_triIdxArr[ idx ]));
        	return (*this->_vertices)[ uidx ];
        }

        //! @copydoc IndexedTriMesh::getIndexedTriangle
        IndexedTriangle<uint32_t> getIndexedTriangle(size_t i) const {
        	const uint32_t idx = static_cast<uint32_t>(this->_stride*i); // this is the unmasked idx
        	const S v1 = *((S*)&(this->_triIdxArr[ idx ]));
        	const S v2 = *((S*)&(this->_triIdxArr[ idx+this->_idxsize ]));
        	const S v3 = *((S*)&(this->_triIdxArr[ idx+2*this->_idxsize ]));
        	return IndexedTriangle<uint32_t>(v1,v2,v3);
        }

        //! @copydoc TriMesh::getTriangle
        Triangle<double> getTriangle(size_t i) const {
            using namespace rw::math;
            const uint32_t idx = this->_stride*(uint32_t)i; // this is the unmasked idx
            const S v0idx = *((S*)&(this->_triIdxArr[ idx ]));
            const S v1idx = *((S*)&(this->_triIdxArr[ idx+this->_idxsize ]));
            const S v2idx = *((S*)&(this->_triIdxArr[ idx+2*this->_idxsize ]));
            const Vector3D<T> &v0 = (*this->_vertices)[v0idx];
            const Vector3D<T> &v1 = (*this->_vertices)[v1idx];
            const Vector3D<T> &v2 = (*this->_vertices)[v2idx];

            if( ::boost::is_same<float, T>::value ){
                return Triangle<double>(cast<double>(v0),cast<double>(v1),cast<double>(v2));
            } else {
                return Triangle<double>(
                        *(reinterpret_cast<const Vector3D<double>* >(&v0)),
                        *(reinterpret_cast<const Vector3D<double>* >(&v1)),
                        *(reinterpret_cast<const Vector3D<double>* >(&v2)));
            }
        }

        //! @copydoc TriMesh::getTriangle
        void getTriangle(size_t i, Triangle<double>& dst) const {
            using namespace rw::math;
            const uint32_t idx = this->_stride*(uint32_t)i; // this is the unmasked idx
            const S v0idx = *((S*)&(this->_triIdxArr[ idx ]));
            const S v1idx = *((S*)&(this->_triIdxArr[ idx+this->_idxsize ]));
            const S v2idx = *((S*)&(this->_triIdxArr[ idx+2*this->_idxsize ]));

            if( ::boost::is_same<float, T>::value ){
                dst[0] = cast<double>( (*this->_vertices)[v0idx] );
                dst[1] = cast<double>( (*this->_vertices)[v1idx] );
                dst[2] = cast<double>( (*this->_vertices)[v2idx] );
            } else {
                dst[0] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v0idx]));
                dst[1] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v1idx]));
                dst[2] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v2idx]));
            }
        }

        //! @copydoc TriMesh::getTriangle
        void getTriangle(size_t i, Triangle<float>& dst) const {
            using namespace rw::math;
            const uint32_t idx = this->_stride*(uint32_t)i; // this is the unmasked idx
            const S v0idx = *((S*)&(this->_triIdxArr[ idx ]));
            const S v1idx = *((S*)&(this->_triIdxArr[ idx+this->_idxsize ]));
            const S v2idx = *((S*)&(this->_triIdxArr[ idx+2*this->_idxsize ]));
            if( ::boost::is_same<float, T>::value ){
                //dst[0] = (*this->_vertices)[v0idx ];
                dst[0] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v0idx]));
                dst[1] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v1idx]));
                dst[2] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v2idx]));
            } else {
                dst[0] = cast<float>( (*this->_vertices)[v0idx] );
                dst[1] = cast<float>( (*this->_vertices)[v1idx] );
                dst[2] = cast<float>( (*this->_vertices)[v2idx] );
            }
        }

        /**
		 * @brief Add indexed triangle to the triangle mesh.
         * @param triangle [in] the indexed triangle to add.
         */
		void add(const tri_type& triangle){
			_triangles->push_back(triangle);
			this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		}

		/**
		 * @brief get vertex i of triangle at index triIdx.
		 * @param i [in] should be in interval [0;2]
		 * @param triIdx [in] index of triangle in the triangle mesh
		 * @return a reference to the given vertex.
		 */
		const rw::math::Vector3D<T>& getTriVertex(size_t i, size_t triIdx) const {
			const tri_type& tri = (*_triangles)[triIdx];
			return (*this->_vertices)[tri.getVertexIdx(i)];
		}

		/**
		 * @brief get vertex i of triangle at index triIdx.
		 * @param i [in] should be in interval [0;2]
		 * @param triIdx [in] index of triangle in the triangle mesh
		 * @return a reference to the given vertex.
		 */
		rw::math::Vector3D<T>& getTriVertex(size_t i, size_t triIdx) {
			const tri_type& tri = (*_triangles)[triIdx];
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
         * @brief get indexed triangle at index i
		 * @return a reference to the triangle.
         */
		tri_type& operator[](int i){
            return (*_triangles)[i];
        }

        /**
         * @brief get indexed triangle at index i
		 * @return a reference to the triangle.
         */
        const tri_type& operator[](int i) const {
            return (*_triangles)[i];
        }

        /**
         * @brief calculate area of triangle at index \b triIdx
         * @return the area.
         */
		T calcFaceArea(size_t triIdx){
			using namespace rw::math;
			const tri_type& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return ( cross( v0-v1 , v0-v2 ) ).norm2()/2;
		}

        /**
         * @brief calculate centroid of triangle at index \b triIdx
         * @return the centroid.
         */
		rw::math::Vector3D<T> calcFaceCentroid(size_t triIdx){
			using namespace rw::math;
			const tri_type& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return (v0+v1+v2)/3;
		}

        /**
         * @brief calculate face normal of triangle at index \b triIdx
         * @return the face normal.
         */
		rw::math::Vector3D<T> calcFaceNormal(size_t triIdx) const {
			using namespace rw::math;
			const tri_type& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			Vector3D<T> n = cross(v1-v0,v2-v0);
	        return normalize(n);
		}

		/**
		 * @brief Set the number of triangles in the mesh.
		 * @param nsize [in] the new size of the mesh.
		 */
		void resize(size_t nsize){
			if(_triangles!=NULL)
				_triangles->resize(nsize);
			if(_triangles!=NULL && _triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		}

		// inherited from IndexedTriMesh
		//! @copydoc IndexedTriMesh::getNrTris
        int getNrTris() const{ return (int)_triangles->size(); };

		// Inherited from TriMesh
		//! @copydoc TriMesh::getSize
		size_t getSize() const {return _triangles->size(); }

		//! @copydoc TriMesh::clone
		TriMesh::Ptr clone() const {
			return rw::common::ownedPtr( new IndexedTriMeshN0(*this) );
		}

	};

	//! @brief The type for a IndexedTriMesh with no normals of double value type and uint16_t as index type.
    typedef IndexedTriMeshN0<> IndexedTriMeshN0D;
    //! @brief The type for a IndexedTriMesh with no normals of float value type and uint16_t as index type.
    typedef IndexedTriMeshN0<float> IndexedTriMeshN0F;

	// @}
}
} // geometry
#endif /*RW_GEOMETRY_INDEXEDTRIMESH_HPP_*/

