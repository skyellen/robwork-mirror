#ifndef RW_GEOMETRY_PLAINTRIMESH_HPP_
#define RW_GEOMETRY_PLAINTRIMESH_HPP_

#include "TriMesh.hpp"

namespace rw {
namespace geometry {

	/**
	 * @brief
	 */
	template <class TRI>
	class PlainTriMesh: public TriMesh<typename TRI::value_type> {
	private:
		std::vector<TRI> _triangles;

	public:

	    typedef typename TRI::value_type value_type;
		/**
		 * @brief constructor
		 */
		PlainTriMesh(int initSize=0):
		    _triangles(initSize)
		{}

		/**
		 * @brief add a triangle to the triangle mesh.
		 */
		void add(const TRI& triangle){
			_triangles.push_back(triangle);
		}

		/**
		 * @brief
		 */
		std::vector<TRI>& getTriangles(){
			return _triangles;
		}

		/**
		 * @brief returns triangle at index i
		 */
		const TRI& operator[](size_t i) const {
			return _triangles[i];
		}

		/**
		* @brief returns triangle at index i
		*/
		TRI& operator[](size_t i) {
		    return _triangles[i];
		}

		// Inherited from TriMesh
		/**
		 * @copydoc TriMesh::getTriangle
		 */
		TriangleN0<value_type> getTriangle(size_t idx) const {
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
