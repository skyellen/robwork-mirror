#ifndef RW_GEOMETRY_TRIANGLEUTIL_HPP_
#define RW_GEOMETRY_TRIANGLEUTIL_HPP_

#include "Triangle.hpp"
#include "TriMesh.hpp"
#include "IndexedTriMesh.hpp"
#include "PlainTriMesh.hpp"

#include <rw/math/MetricUtil.hpp>
#include <boost/foreach.hpp>

namespace rw { namespace geometry {

	class TriangleUtil
    {
	public:
		/**
		 * @brief takes a general triangle mesh and creates a indexed 
		 * triangle mesh. All data is copied.  
		 * @param triMesh [in] the tri mesh that is to be converted
		 * @param epsilon [in] if two vertices re closer than epsilon they
		 * are considered the equal.
		 */
		template <class T, TriType TRI>
		static IndexedTriMesh<T, TRI>* ToIndexedTriMesh( const TriMesh<T>& triMesh, 
														 double epsilon=0.0000001)
		{
			using namespace rw::math;
			// create vertice array
			std::vector<Vector3D<T> > *vertices = 
				new std::vector<Vector3D<T> >();
			std::vector<IndexedTriangle<TRI> > *triangles = 
				new std::vector<IndexedTriangle<TRI> >();
			// for each triangle check if any of the triangle vertices are allready in vertices array
			for(size_t i = 0; i<triMesh.getSize(); i++){
				int v0=-1,v1=-1,v2=-1;
				Triangle<T,N0> tri = triMesh.getTriangle(i);
				for(size_t j=0; j<vertices->size(); j++){
					Vector3D<T> &v = (*vertices)[j];
					if( MetricUtil::dist2<T>(v, tri.getVertex(0))<epsilon )
						v0 = j;
					if( MetricUtil::dist2<T>(v, tri.getVertex(1))<epsilon )
						v1 = j;
					if( MetricUtil::dist2<T>(v, tri.getVertex(2))<epsilon )
						v2 = j;				
				}
				if( v0<0 ){
					v0 = vertices->size();
					vertices->push_back( tri.getVertex(0) );
				}
				if( v1<0 ){
					v1 = vertices->size();
					vertices->push_back( tri.getVertex(1) );
				}
				if( v2<0 ){
					v2 = vertices->size();
					vertices->push_back( tri.getVertex(2) );
				}
				triangles->push_back( IndexedTriangle<TRI>(v0,v1,v2) );
			}
			std::cout << "Indexed triangle mesh created: " << vertices->size() 
					  << " " << triangles->size() << std::endl;
			for(size_t i=0; i<vertices->size(); i++)
				std::cout << (*vertices)[i] << std::endl;
			return new IndexedTriMesh<T, TRI>(vertices, triangles);
		}
		
			
		/**
		 * @brief takes a indexed triangle mesh and creates a generel 
		 * triangle mesh. All data is copied.  
		 * @param triMesh [in] the tri mesh that is to be converted
		 */	
		//template <class TRI=Triangle<double> >
		//static TriMesh<TRI>* toTriMesh( const IndexedTriMesh<TRI>& iTriMesh );
		
		
		//template <class TRI=Triangle<double> >
		//static TriMesh<TRI>* toMesh( const IndexedTriMesh<TRI>& iTriMesh );
			
		// calculate and add vertex normals from/to TriMesh
		template <class T>
		static void CalcVertexNormals( const TriMesh<T>& triMesh,
									   PlainTriMesh<T,N3>& result)
		{
			
		}
		
		// calculate and add vertex normals from/to TriMesh
		template <class T, TriType TRI>
		static void CalcVertexNormals( const IndexedTriMesh<T, TRI>& triMesh,
									   IndexedTriMesh<T, N3>& result)
		{
			
		}
	
	};

}} // end namespaces

#endif // end include guard
