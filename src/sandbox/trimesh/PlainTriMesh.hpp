#ifndef RW_GEOMETRY_PLAINTRIMESH_HPP_
#define RW_GEOMETRY_PLAINTRIMESH_HPP_

#include "TriMesh.hpp"

namespace rw {
namespace geometry {

	/**
	 * @brief 
	 */
	template <class T=double, TriType TRI=N0>
	class PlainTriMesh: public TriMesh<T> {
	private:
		std::vector<Triangle<T, TRI> > _triangles;
		
	public:
		
		/**
		 * @brief constructor
		 */
		PlainTriMesh(){}

		/**
		 * @brief add a triangle to the triangle mesh.
		 */
		void add(const Triangle<T,TRI>& triangle){
			_triangles.push_back(triangle);
		}

		/**
		 * @brief  
		 */
		std::vector<Triangle<T,TRI> >& getTriangles(){
			return _triangles;
		}
		
		// Inherited from TriMesh
		/**
		 * @copydoc TriMesh::getTriangle  
		 */
		Triangle<T,N0> getTriangle(size_t idx) const {
			return _triangles[idx];
		}

		/**
		 * @copydoc TriMesh::getSize  
		 */
		size_t getSize() const {
			return _triangles.size();
		}

	};

} // geometry
} // rw

#endif /*TRIMESH_HPP_*/
