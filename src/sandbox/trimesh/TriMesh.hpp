#ifndef RW_GEOMETRY_TRIMESH_HPP_
#define RW_GEOMETRY_TRIMESH_HPP_

#include "Triangle.hpp"

namespace rw {
namespace geometry {

	/**
	 * @brief interface of a triangle mesh. The interface defines a way to get
	 * triangles from a triangle array/mesh.
	 */
	template <class T=double>
	class TriMesh {
	public:
		typedef T value_type;

		/**
		 * @brief destructor
		 */
		virtual ~TriMesh(){};

		/**
		 * @brief gets the triangle at index idx.
		 */
		virtual TriangleN0<T> getTriangle(size_t idx) const = 0;

		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t getSize() const = 0;

		virtual size_t size() const{
		    return getSize();
		}
	};

} // geometry
} // rw


#endif /*TRIMESH_HPP_*/
