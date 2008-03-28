#ifndef INDEXEDTRIMESH_HPP_
#define INDEXEDTRIMESH_HPP_


#include <rw/math/Vector3D.hpp>
#include "TriMesh.hpp"
#include "Triangle.hpp"
#include "IndexedTriangle.hpp"


/**
 * 
 * @brief an Indexed Triangle mesh 
 * 
 */
template <class T, TriType TRI>
class IndexedTriMesh: public TriMesh<T> {
public:
	typedef std::vector<rw::math::Vector3D<T> > VertexArray;
	typedef std::vector<rw::math::Vector3D<T> > TriangleArray;
	
private:
	typedef T primType;

	std::vector<IndexedTriangle<TRI> > *_triangles;
	std::vector<rw::math::Vector3D<T> > *_vertices;
	std::vector<rw::math::Vector3D<T> > *_faceNormals;
	std::vector<rw::math::Vector3D<T> > *_normals;
	
public:
	
	typedef IndexedTriangle<TRI> value_type;
	
	IndexedTriMesh():
		_triangles( new TriangleArray() ),
		_vertices( new VertexArray() ),
		_faceNormals( new VertexArray() )
	{};
	
	IndexedTriMesh(std::vector<rw::math::Vector3D<T> > *vertices,
				   std::vector<IndexedTriangle<TRI> > *triangles):
					   _triangles(triangles),
					   _vertices(vertices)
	{
		
	};

	virtual ~IndexedTriMesh(){};
	
	void add(const IndexedTriangle<TRI>& triangle){
		_triangles->push_back(triangle);
	}
	
	const rw::math::Vector3D<primType>& getVertex(size_t i){
		return (*_vertices)[i];
	}

	const rw::math::Vector3D<primType>& getVertex(size_t i, size_t triIdx){
		const IndexedTriangle<TRI>& tri = (*_triangles)[triIdx];
		return (*_vertices)[tri.getVertexIdx(i)];
	}
	
	const rw::math::Vector3D<primType>& getVertexNormal(size_t i){
		return (*_normals)[i];
	}

	const rw::math::Vector3D<primType>& getFaceNormal(size_t i){
		return (*_faceNormals)[i];
	}

	const std::vector<rw::math::Vector3D<primType> >& getVertices() const {
		return *_vertices;
	}
	
	const std::vector<rw::math::Vector3D<primType> >& getFaceNormals(){
		return *_faceNormals;
	}
	
	const std::vector<rw::math::Vector3D<primType> >& getVertexNormals(){
		return *_normals;
	}
	
	std::vector<IndexedTriangle<TRI> >& getTriangles(){
		return *_triangles;
	}
	
	T calcFaceArea(size_t triIdx){
		using namespace rw::math;
		const IndexedTriangle<TRI>& tri = (*_triangles)[triIdx];
		const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
		const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
		const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );
		
		return (cross(v0-v1,v0-v2) ).norm2/2;		
	}

	rw::math::Vector3D<T> calcFaceCentroid(size_t triIdx){
		using namespace rw::math;
		const IndexedTriangle<TRI>& tri = (*_triangles)[triIdx];
		const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
		const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
		const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );
		
		return (v0+v1+v2)/3;		
	}
	
	rw::math::Vector3D<T> calcFaceNormal(size_t triIdx) const {
		using namespace rw::math;
		const IndexedTriangle<TRI>& tri = (*_triangles)[triIdx];
		const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
		const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
		const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );
		
		Vector3D<T> n = cross(v1-v0,v2-v0);
        return normalize(n);
	}
	
	Triangle<T,TRI> getTriangle(const IndexedTriangle<N1>& idxTri) const {
		
	}
	
	// Inherited from TriMesh
	Triangle<T> getTriangle(size_t idx) const {
		using namespace rw::math;
		const IndexedTriangle<TRI>& tri = (*_triangles)[idx];
		const Vector3D<T> &v0( (*_vertices)[tri.getVertexIdx(0) ] );
		const Vector3D<T> &v1( (*_vertices)[tri.getVertexIdx(1) ] );
		const Vector3D<T> &v2( (*_vertices)[tri.getVertexIdx(2) ] );
		Triangle<T> tri1 = Triangle<T>(v0,v1,v2);
		return tri1;
	}
	
	size_t getSize() const {
		return _triangles->size();
	}
	
	
};


#endif /*INDEXEDTRIMESH_HPP_*/
