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
    		_mask = 0;
    		for(uint8_t i=0;i<idxsize;i++)
    			_mask |= 0xFF<<(i*8);
    		//std::cout << "MASK: " << std::hex << _mask << std::endl;
		}

    	//! @brief destructor
    	virtual ~IndexedTriMesh(){
     	}

    public:

    	//! @brief get array of normals
        const std::vector<rw::math::Vector3D<T> >& getNormals() const{
        	RW_ASSERT(_normals);
        	return *_normals;
        }

        //! @brief get array of normals
        std::vector<rw::math::Vector3D<T> >& getNormals(){
        	RW_ASSERT(_normals);
        	return *_normals;
        };

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

        //! @copydoc TriMesh::size
        virtual size_t size() const { return getNrTris();};

        //! @brief return a vertice indexed into the vertice list
        const rw::math::Vector3D<T>& getVertex(size_t i) const{ return _vertices->at(i);};

        //! @brief return a vertice indexed into the vertice list
        rw::math::Vector3D<T>& getVertex(size_t i){ return _vertices->at(i);};

        bool hasVertexNormals(){ return _normals->size() == _vertices->size(); };

        bool hasFaceNormals(){ return _normals->size() == getNrTris(); };

        const rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx) const{
            const uint32_t idx = _stride*i+vidx*_idxsize; // this is the unmasked idx
            const uint32_t uidx =  *((uint32_t*)&(_triIdxArr[ idx ]))&_mask; // now we mask the idx
            return (*_normals)[ uidx ];
        }

        rw::math::Vector3D<T>& getVertexNormal(size_t i, VertexIdx vidx){
            const uint32_t idx = _stride*i+vidx*_idxsize; // this is the unmasked idx
            const uint32_t uidx =  *((uint32_t*)&(_triIdxArr[ idx ]))&_mask; // now we mask the idx
            return (*_normals)[ uidx ];
        }


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
        	const uint32_t idx = (uint32_t)(_stride*i+vidx*_idxsize); // this is the unmasked idx
        	const uint32_t uidx =  *((uint32_t*)&(_triIdxArr[ idx ]))&_mask; // now we mask the idx
        	return (*_vertices)[ uidx ];
        }

        /**
         * @brief get the indexed triangle at index \b i
         * @param i [in] the index of the triangle
         * @return an indexed triangle
         */
        IndexedTriangle<uint32_t> getIndexedTriangle(size_t i) const {
        	const uint32_t idx = (uint32_t)(_stride*i); // this is the unmasked idx
        	const uint32_t v1 = *((uint32_t*)&(_triIdxArr[ idx ]));
        	const uint32_t v2 = *((uint32_t*)&(_triIdxArr[ idx+_idxsize ]));
        	const uint32_t v3 = *((uint32_t*)&(_triIdxArr[ idx+2*_idxsize ]));
        	/*if((v1&_mask)>(uint32_t)getNrTris()){
        		std::cout << "index: " << idx << ", " << i<< std::endl;
        		std::cout << "stride: " << ((int)_stride) << std::endl;
        		std::cout << "mask: " << _mask << std::endl;
        		std::cout << v1 << ", " << v2 << ", " << v3 << std::endl;
        		std::cout << (v1&_mask) << ", " << (v2&_mask) << ", " << (v3&_mask) << " <<< " << getNrTris() << std::endl;
        	}*/
        	return IndexedTriangle<uint32_t>(v1&_mask,v2&_mask,v3&_mask);
        }

        /**
         * @brief The number of triangles in the mesh
         * @return nr of triangles
         */
        virtual int getNrTris() const = 0 ;

        /**
         * @copydoc GeometryData::getType
         */
        GeometryData::GeometryType getType() const{
            return GeometryData::IdxTriMesh;
        };

        void* getIndices(){return (void*)_triIdxArr;};

        // Inherited from TriMesh
        //! @copydoc IndexedTriMesh::getTriangle
        Triangle<double> getTriangle(size_t i) const {
            using namespace rw::math;
            const uint32_t idx = _stride*(uint32_t)i; // this is the unmasked idx
            const uint32_t v0idx = *((uint32_t*)&(_triIdxArr[ idx ]));
            const uint32_t v1idx = *((uint32_t*)&(_triIdxArr[ idx+_idxsize ]));
            const uint32_t v2idx = *((uint32_t*)&(_triIdxArr[ idx+2*_idxsize ]));
            const Vector3D<T> &v0 = (*this->_vertices)[v0idx&_mask ] ;
            const Vector3D<T> &v1 = (*this->_vertices)[v1idx&_mask ] ;
            const Vector3D<T> &v2 = (*this->_vertices)[v2idx&_mask ] ;

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
            const uint32_t idx = _stride*(uint32_t)i; // this is the unmasked idx
            const uint32_t v0idx = *((uint32_t*)&(_triIdxArr[ idx ]));
            const uint32_t v1idx = *((uint32_t*)&(_triIdxArr[ idx+_idxsize ]));
            const uint32_t v2idx = *((uint32_t*)&(_triIdxArr[ idx+2*_idxsize ]));

            if( ::boost::is_same<float, T>::value ){
                dst[0] = cast<double>( (*this->_vertices)[v0idx&_mask ] ) ;
                dst[1] = cast<double>( (*this->_vertices)[v1idx&_mask ] ) ;
                dst[2] = cast<double>( (*this->_vertices)[v2idx&_mask ] ) ;
            } else {
                dst[0] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v0idx&_mask ]));
                dst[1] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v1idx&_mask ]));
                dst[2] = *(reinterpret_cast<const Vector3D<double>* >(&(*this->_vertices)[v2idx&_mask ]));
            }
        }

        //! @copydoc TriMesh::getTriangle
        void getTriangle(size_t i, Triangle<float>& dst) const {
            using namespace rw::math;
            const uint32_t idx = _stride*(uint32_t)i; // this is the unmasked idx
            const uint32_t v0idx = *((uint32_t*)&(_triIdxArr[ idx ]));
            const uint32_t v1idx = *((uint32_t*)&(_triIdxArr[ idx+_idxsize ]));
            const uint32_t v2idx = *((uint32_t*)&(_triIdxArr[ idx+2*_idxsize ]));
            if( ::boost::is_same<float, T>::value ){
                //dst[0] = (*this->_vertices)[v0idx ];
                dst[0] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v0idx&_mask ]));
                dst[1] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v1idx&_mask ]));
                dst[2] = *(reinterpret_cast<const Vector3D<float>* >(&(*this->_vertices)[v2idx&_mask ]));
            } else {
                dst[0] = cast<float>( (*this->_vertices)[v0idx&_mask ] ) ;
                dst[1] = cast<float>( (*this->_vertices)[v1idx&_mask ] ) ;
                dst[2] = cast<float>( (*this->_vertices)[v2idx&_mask ] ) ;
            }
        }
    private:
        IndexedTriMesh(IndexedTriMesh<T>& mesh){} // this is illegal


    protected:
        //! @brief pointer to vertice array
		VertexArray *_vertices;
		rw::common::Ptr<VertexArray> _vertPtr;
		//! @brief pointer to normal array
		VertexArray *_normals;
		rw::common::Ptr<VertexArray> _normPtr;
		//! @brief pointer to indice array
		uint8_t *_triIdxArr;
		//! @brief sets the indice array
		void setTriArray(uint8_t *triarray){ _triIdxArr = triarray;};
    private:
		const uint8_t _stride, _idxsize;
		uint32_t _mask;
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
	 *
	 * @brief an Indexed Triangle mesh with zero normals
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
		//! smart pointer type of this class
		typedef rw::common::Ptr<IndexedTriMeshN0<T, S> > Ptr;
	private:
	    //typedef value_type T;
		rw::common::Ptr<TriangleArray> _triangles;

	public:

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0():
			IndexedTriMesh<T>(
			        rw::common::ownedPtr( new VertexArray() ),
			        rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
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
					rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
		{
			//this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};


		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in]
		 * @param normals [in]
		 */
		IndexedTriMeshN0(
				rw::common::Ptr<std::vector<rw::math::Vector3D<T> > > vertices,
				rw::common::Ptr<std::vector<rw::math::Vector3D<T> > > normals):
			IndexedTriMesh<T>(
					vertices,
					normals,
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( rw::common::ownedPtr( new TriangleArray() ) )
		{
			//this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0(
				rw::common::Ptr<std::vector<rw::math::Vector3D<T> > > vertices,
				rw::common::Ptr<std::vector<rw::math::Vector3D<T> > > normals,
				rw::common::Ptr<TriangleArray> triangles):
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

		/**
		 * @brief constructor
		 * @param vertices
		 * @param triangles
		 * @return
		 */
		IndexedTriMeshN0(rw::common::Ptr<VertexArray> vertices, rw::common::Ptr<TriangleArray> triangles):
			IndexedTriMesh<T>(
					vertices,
					rw::common::ownedPtr( new VertexArray() ),
					(uint8_t)sizeof(TRI),
					(uint8_t)sizeof(S)),
			_triangles( triangles )
		{
			RW_ASSERT( _triangles );
			if(triangles!=NULL && triangles->size()>0)
				this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		};

		IndexedTriMeshN0(const IndexedTriMeshN0& mesh):
		    IndexedTriMesh<T>(
		            rw::common::ownedPtr(new VertexArray( mesh.getVertices() )),
		            rw::common::ownedPtr(new VertexArray( mesh.getNormals() )),
		            (uint8_t)sizeof(TRI),
		            (uint8_t)sizeof(S)
		    ),
		    _triangles( rw::common::ownedPtr(new TriangleArray(mesh.getTriangles()) ) )
		{
            if(_triangles!=NULL && _triangles->size()>0)
                this->setTriArray((uint8_t*)&((*_triangles)[0].getVertexIdx(0)));
		}

		/**
		 * @brief destructor
		 */
		virtual ~IndexedTriMeshN0(){

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
         * @brief get indexed triangle at index i
         */
        TRI& operator[](int i){
            return (*_triangles)[i];
        }

        /**
         * @brief get indexed triangle at index i
         */
        const TRI& operator[](int i) const {
            return (*_triangles)[i];
        }

        /**
         * @brief calculate area of triangle at index \b triIdx
         */
		T calcFaceArea(size_t triIdx){
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return ( cross( v0-v1 , v0-v2 ) ).norm2()/2;
		}

        /**
         * @brief calculate centroid of triangle at index \b triIdx
         */
		rw::math::Vector3D<T> calcFaceCentroid(size_t triIdx){
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			return (v0+v1+v2)/3;
		}

        /**
         * @brief calculate face normal of triangle at index \b triIdx
         */
		rw::math::Vector3D<T> calcFaceNormal(size_t triIdx) const {
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*this->_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*this->_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*this->_vertices)[tri.getVertexIdx(2) ] );

			Vector3D<T> n = cross(v1-v0,v2-v0);
	        return normalize(n);
		}

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
	// @}

    typedef IndexedTriMeshN0<> IndexedTriMeshN0D;
    typedef IndexedTriMeshN0<float> IndexedTriMeshN0F;

#ifdef RW_USE_DEPRECATED
	typedef rw::common::Ptr<IndexedTriMeshN0D> IndexedTriMeshN0DPtr;

	typedef rw::common::Ptr<IndexedTriMeshN0<float> > IndexedTriMeshN0FPtr;
#endif
}
} // geometry


#endif /*INDEXEDTRIMESH_HPP_*/
