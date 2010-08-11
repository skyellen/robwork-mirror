#ifndef RW_GEOMETRY_INDEXEDTRIARRAY_HPP_
#define RW_GEOMETRY_INDEXEDTRIARRAY_HPP_

#include <rw/geometry/TriMesh.hpp>

namespace rw {
namespace geometry {

	/**
	 * @brief this class is a proxy to a triangle mesh where the triangles
	 * can be indirectly copied in an efficient manner.
	 *
	 * Each "virtual" triangle index is mapped to the actual triangle index.
	 * By rearanging the mapping the order of the triangles in the mesh are
	 * rearanged to the proxy user, without changing the actual triangle mesh.
	 *
	 */
	template<class T=size_t>
	class IndexedTriArray : public rw::geometry::TriMesh {
	private:
		const rw::geometry::TriMesh *_objArr;
		std::vector<T>& _idxArr;
		size_t _first,_last;
	public:

		/**
		 * @brief constructor
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 * @param idxArr [in] the index mapping
		 */
		IndexedTriArray(const TriMesh *objArr,
						std::vector<T>& idxArr):
			_objArr(objArr),
			_idxArr(idxArr),
			_first(0),
			_last(idxArr.size())
		{
		}

		/**
		 * @brief constructor
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 * @param idxArr [in] the index mapping
		 */
		IndexedTriArray(const TriMesh *objArr,
						std::vector<T>& idxArr,
						size_t first,
						size_t last):
			_objArr(objArr),
			_idxArr(idxArr),
			_first(first),
			_last(last)
		{
		}


		//! @brief destructor
		virtual ~IndexedTriArray(){}

		//! @brief get the index mapping
		const std::vector<T>& getIndexes() const{
			return _idxArr;
		}

		/**
		 * @brief sorts the triangles in the range [first,last[. the vertices of the triangles
		 * are projected onto the axis and then sorted in regard to the projected value
		 * from small to high
		 * @param axis
		 */
		void sortAxis(int axis){
			struct TrisIdxSort
			{
				TrisIdxSort(
					 const int splitAxis,
					 const rw::math::Transform3D<T>& t3d,
					 const IdxTriMesh& mesh):
					_splitAxis(splitAxis),
					_t3d(t3d),
					_mesh(mesh)
				{}

				bool operator()(const int& i0, const int& i1) {
					using namespace rw::math;
					using namespace rw::geometry;
					const std::vector<Vector3D<T> > &verts = _mesh.getVertices();
					const IndexedTriangle<T> &tri = _mesh[ i0 ];
					Vector3D<T> c0 = _t3d*((verts[ tri[0] ]+verts[ tri[1] ]+verts[ tri[2] ])/3.0);
					const IndexedTriangle<T> &t1 = _mesh[ i1 ];
					Vector3D<T> c1 = _t3d*((verts[ t1[0] ]+verts[ t1[1] ]+verts[ t1[2] ])/3.0);

					return  c0(_splitAxis)<c1(_splitAxis);
				}
				const int _splitAxis;
				const rw::math::Transform3D<T> _t3d;
				const IdxTriMesh& _mesh;
			};



		}


		IndexedTriArray getSubRange(size_t first, size_t last){
			return IndexedTriArray(_first+first, _first+last);
		}


		size_t getGlobalIndex(int idx){ return _first+idx; }

		// **** inherited from trimesh
		//! @copydoc TriMesh::operator[]
		rw::geometry::Triangle<> operator[](size_t i) const {
			return _objArr->getTriangle(_idxArr[i]);
		}

		//! @copydoc TriMesh::getTriangle
		rw::geometry::Triangle<> getTriangle(size_t idx) const{
			return (*this)[idx];
		}

		//! @copydoc TriMesh::getType
		rw::geometry::GeometryData::GeometryType getType(){
			return GeometryData::UserType;
		}

		//! @copydoc TriMesh::getSize
		size_t getSize() const{
			return _last-_first;
		}




	};

}
}

#endif /*INDEXEDARRAY_HPP_*/
