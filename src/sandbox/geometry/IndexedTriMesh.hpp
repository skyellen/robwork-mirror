/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_GEOMETRY_INDEXEDTRIMESH_HPP_
#define RW_GEOMETRY_INDEXEDTRIMESH_HPP_

#include <rw/math/Vector3D.hpp>
#include "TriMesh.hpp"
#include "Triangle.hpp"
#include "IndexedTriangle.hpp"

namespace rw {
namespace geometry {
namespace sandbox {
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

        virtual const std::vector<rw::math::Vector3D<T> >& getVertices() const = 0;

        virtual std::vector<rw::math::Vector3D<T> >& getVertices() = 0;

        virtual const rw::math::Vector3D<T>& getVertex(size_t i) const = 0;

        virtual rw::math::Vector3D<T>& getVertex(size_t i) = 0;

        /**
         * @brief retrieve a vertex of a specific triangle
         * @param i [in] the index of the triangle
         * @param vidx [in] the index of the triangle vertex
         * @return the vertex with index \b vidx of the triangle
         */
        virtual const rw::math::Vector3D<T>& getTriVertex(size_t i, VertexIdx vidx) const = 0;

        /**
         * @brief retrieve a vertex of a specific triangle
         * @param i [in] the index of the triangle
         * @param vidx [in] the index of the triangle vertex
         * @return the vertex with index \b vidx of the triangle
         */
        virtual rw::math::Vector3D<T>& getTriVertex(size_t i, VertexIdx vidx) = 0;

        /**
         * @brief get vertex at index i
         */
        virtual IndexedTriangle<T>& operator[](int i) = 0;

        /**
         * @brief get vertex at index i
         */
        virtual const IndexedTriangle<T>& operator[](int i) const = 0;

        virtual int getNrTris() const = 0 ;

        GeometryData::GeometryType getType(){
            return GeometryData::IdxTriMesh;
        };

    };

	/**
	 *
	 * @brief an Indexed Triangle mesh
	 *
	 */
	template <class T=double>
	class IndexedTriMeshN0: public IndexedTriMesh<T> {
	public:
	    typedef T value_type;
	    typedef IndexedTriangleN0<T> tri_type;
	    typedef IndexedTriangleN0<T> TRI;
	    typedef std::vector<rw::math::Vector3D<value_type> > VertexArray;
		typedef std::vector<TRI> TriangleArray;

	private:
	    //typedef value_type T;

		TriangleArray *_triangles;
		VertexArray *_vertices;
		VertexArray *_normals;

		unsigned int _stride;
		unsigned char *_triIdxArr;

	public:

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0():
			_triangles( new TriangleArray() ),
			_vertices( new VertexArray() ),
			_stride( sizeof(TRI) )
		{
		};

		/**
		 * @brief constructor - ownership of the vertice array is taken
		 * @param vertices [in]
		 */
		IndexedTriMeshN0(std::vector<rw::math::Vector3D<T> > *vertices):
			_triangles( new TriangleArray() ),
			_vertices( vertices ),
			_stride( sizeof(TRI) )
		{

		};

		/**
		 * @brief constructor
		 */
		IndexedTriMeshN0(VertexArray *vertices, TriangleArray *triangles):
           _triangles(triangles),
           _vertices(vertices),
           _stride( sizeof(TRI) )
		{
			_triIdxArr = (unsigned char*) &((*_triangles)[0].getVertexIdx(0));
		};

		/**
		 * @brief destructor
		 */
		virtual ~IndexedTriMeshN0(){};

		/**
		 * @brief add indexed triangle to the triangle mesh.
		 */
		void add(const TRI& triangle){
			_triangles->push_back(triangle);
			_triIdxArr = (unsigned char*) &((*_triangles)[0].getVertexIdx(0));
		}

		/**
		 * @brief
		 */
		rw::math::Vector3D<T>& getVertex(size_t i){
			return (*_vertices)[i];
		}

		/**
         * @brief
         */
        const rw::math::Vector3D<T>& getVertex(size_t i) const {
            return (*_vertices)[i];
        }

		/**
		 * @brief get vertex i of triangle at index triIdx.
		 * @param i [in] should be in interval [0;2]
		 * @param triIdx [in] index of triangle in the triangle mesh
		 */
		const rw::math::Vector3D<T>& getVertex(size_t i, size_t triIdx) const {
			const TRI& tri = (*_triangles)[triIdx];
			return (*_vertices)[tri.getVertexIdx(i)];
		}

        const rw::math::Vector3D<T>& getTriVertex(size_t i, VertexIdx vidx) const{
        	return (*_vertices)[  *((int*)&(_triIdxArr[_stride*i+vidx*sizeof(int)]))  ];
        }

        rw::math::Vector3D<T>& getTriVertex(size_t i, VertexIdx vidx){
        	return (*_vertices)[  *((int*)&(_triIdxArr[_stride*i+vidx*sizeof(int)]))  ];
        	//const int vertIdx = (*_triangles)[i].getVertexIdx( vidx );
        	//return (*_vertices)[vertIdx];
        }


        int getNrTris() const{ return _triangles->size(); };


		/**
		 * @brief
		 */
/*		const rw::math::Vector3D<T>& getVertexNormal(size_t i){
			return (*_normals)[i];
		}

		const rw::math::Vector3D<T>& getFaceNormal(size_t i){
			return (*_faceNormals)[i];
		}
*/
		const VertexArray& getVertices() const {
			return *_vertices;
		}

	    VertexArray& getVertices() {
	        return *_vertices;
	    }

		TriangleArray& getTriangles(){
			return *_triangles;
		}

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
			const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );

			return ( cross( v0-v1 , v0-v2 ) ).norm2()/2;
		}

		rw::math::Vector3D<T> calcFaceCentroid(size_t triIdx){
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );

			return (v0+v1+v2)/3;
		}

		rw::math::Vector3D<T> calcFaceNormal(size_t triIdx) const {
			using namespace rw::math;
			const TRI& tri = (*_triangles)[triIdx];
			const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );

			Vector3D<T> n = cross(v1-v0,v2-v0);
	        return normalize(n);
		}

		// Inherited from TriMesh
		TriangleN0<double> getTriangle(size_t idx) const {
			using namespace rw::math;
			const TRI& tri = (*_triangles)[idx];
			const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
			const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
			const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );

			return TriangleN0<double>(cast<double>(v0),cast<double>(v1),cast<double>(v2));
		}

		size_t getSize() const {
			return _triangles->size();
		}


	};
}
} // geometry
} // geometry

#endif /*INDEXEDTRIMESH_HPP_*/
