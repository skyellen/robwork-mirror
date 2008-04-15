#ifndef RW_GEOMETRY_INDEXEDTRIANGLE_HPP_
#define RW_GEOMETRY_INDEXEDTRIANGLE_HPP_

#include <rw/math/Vector3D.hpp>

#include "Triangle.hpp"

namespace rw {
namespace geometry {

	template<TriType T>
	class IndexedTriangle {	
	protected:
		size_t _vertices[3];
		
	public:
		
		
	    //@brief default constructor
		IndexedTriangle(){};
	
	    /**
	     * @brief 
	     */
	    IndexedTriangle(size_t p1, size_t p2, size_t p3) 
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
	        _vertices[0] = f.getVertexIdx(0);
	        _vertices[1] = f.getVertexIdx(1);
	        _vertices[2] = f.getVertexIdx(2);
	    };
	
	    /**
	     * @brief returns the index of vertex i of the triangle
	     */
	    size_t getVertexIdx(size_t i) const {
			return _vertices[i];
		}
	    
		/**
		 * @brief tests wheather the point x is inside the triangle
		 */	
	    template <class A>
		bool isInside(const rw::math::Vector3D<A>& x, 
					  const std::vector<rw::math::Vector3D<A> >& verts){
			using namespace rw::math;
			// calc vectors
			Vector3D<A> v0 = verts[_vertices[2]] - verts[_vertices[0]];
			Vector3D<A> v1 = verts[_vertices[1]] - verts[_vertices[0]];
			Vector3D<A> v2 = x - verts[_vertices[0]];
			// calc dot products
			A dot00 = dot(v0, v0);
			A dot01 = dot(v0, v1);
			A dot02 = dot(v0, v2);
			A dot11 = dot(v1, v1);
			A dot12 = dot(v1, v2);
			// calc barycentric coordinates
			A invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
			A u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			A v = (dot00 * dot12 - dot01 * dot02) * invDenom;
			
			// Check if point is in triangle
			return (u > 0) && (v > 0) && (u + v < 1);
		}

			
	};
	
	template<>
	class IndexedTriangle<N1>: public IndexedTriangle<N0> {
	protected:
		size_t _faceNormal;
	public:
		
	    //@brief default constructor
		IndexedTriangle(){};
	
	    /**
	     * @brief 
	     */
	    IndexedTriangle(size_t p1, size_t p2, size_t p3, size_t n) 
	    {
	    	_vertices[0] = p1;
	    	_vertices[1] = p2;
	    	_vertices[2] = p3;
	    	_faceNormal = n;
	    }
	
	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangle(const IndexedTriangle<N1>& f){
	        _vertices[0] = f.getVertexIdx(0);
	        _vertices[1] = f.getVertexIdx(1);
	        _vertices[2] = f.getVertexIdx(2);
	        _faceNormal = f.getFaceNormalIdx();
	    };
	
	    
	    size_t getFaceNormalIdx() const {
			return _faceNormal;
		}
			
	};
	
	template<>
	class IndexedTriangle<N3>: public IndexedTriangle<N0> {
	protected:
		size_t _normals[3];
	public:
		
	    //@brief default constructor
		IndexedTriangle(){};
	
	    /**
	     * @brief 
	     */
	    IndexedTriangle(size_t p1, size_t p2, size_t p3,
	    				size_t n1, size_t n2, size_t n3):
	    					IndexedTriangle<N0>(p1,p2,p3)
	    {
	    	_normals[0] = n1;
	    	_normals[1] = n2;
	    	_normals[2] = n3;
	    }
	
	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangle(const IndexedTriangle<N3>& f){
	        _vertices[0] = f.getVertexIdx(0);
	        _vertices[1] = f.getVertexIdx(1);
	        _vertices[2] = f.getVertexIdx(2);
	        _normals[0] = f.getVertexNormalIdx(0);
	        _normals[1] = f.getVertexNormalIdx(1);
	        _normals[2] = f.getVertexNormalIdx(2);        
	    };
	
	    
	    size_t getVertexNormalIdx(size_t i) const {
			return _normals[i];
		}
			
	};

} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
