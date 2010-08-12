#ifndef RW_GEOMETRY_INDEXEDTRIARRAY_HPP_
#define RW_GEOMETRY_INDEXEDTRIARRAY_HPP_

#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Transform3D.hpp>

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
		rw::geometry::TriMeshPtr _objArr;
		rw::common::Ptr< std::vector<T> > _idxArr;
		size_t _first,_last;

	public:

		/**
		 * @brief constructor - creates a proxy that initially maps triangles
		 * from [0:0,1:1,2:2,....,i:i]
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 */
		IndexedTriArray(TriMeshPtr objArr):
			_objArr(objArr),
			_idxArr(rw::common::ownedPtr(new std::vector<T>(objArr->getSize()))),
			_first(0),
			_last(objArr->getSize())
		{
			for(size_t i=0;i<_idxArr->size();i++)
				(*_idxArr)[i] = i;
		}

		/**
		 * @brief constructor - creates a proxy where the initial mapping
		 * is determined by \b idxArr.
		 *
		 * idxArr must be same length as the number of triangles.
		 *
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 * @param idxArr [in] the index mapping
		 */
		IndexedTriArray(TriMeshPtr objArr,
						rw::common::Ptr< std::vector<T> > idxArr):
			_objArr(objArr),
			_idxArr(idxArr),
			_first(0),
			_last(idxArr->size())
		{
		}

		/**
		 * @brief constructor - creates a proxy that only references part of
		 * the triangle mesh. the part is specified in the range from
		 * \b first to \b last
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 * @param idxArr [in] the index mapping
		 * @param first [in] the first index
		 */
		IndexedTriArray(TriMeshPtr objArr,
						rw::common::Ptr< std::vector<T> > idxArr,
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
			return *_idxArr;
		}

	private:
		struct TrisIdxSort
		{
			TrisIdxSort(
				 const int splitAxis,
				 const rw::math::Transform3D<>& t3d,
				 const TriMesh& mesh):
				_splitAxis(splitAxis),
				_t3d(t3d),
				_mesh(mesh)
			{}

			bool operator()(const int& i0, const int& i1) {
				using namespace rw::math;
				using namespace rw::geometry;

				const Triangle<> &tri = _mesh.getTriangle(i0);
				Vector3D<> c0 = _t3d*((tri[0]+tri[1]+tri[2])/3.0);
				const Triangle<> &t1 = _mesh.getTriangle(i1);
				Vector3D<> c1 = _t3d*((t1[0]+t1[1]+t1[2])/3.0);

				return  c0(_splitAxis)<c1(_splitAxis);
			}
			const int _splitAxis;
			const rw::math::Transform3D<> _t3d;
			const TriMesh& _mesh;
		};

	public:

		/**
		 * @brief sorts the triangles in the range [first,last[. the vertices of the triangles
		 * are projected onto the axis and then sorted in regard to the projected value
		 * from small to high
		 * @param axis
		 */
		void sortAxis(int axis){
			rw::math::Transform3D<> t3d; // identity
			sortAxis(axis, t3d);

		}

		void sortAxis(int axis, const rw::math::Transform3D<>& t3d){
			std::sort(_idxArr->begin(), _idxArr->end(), TrisIdxSort(axis, t3d, *this));
		}


		IndexedTriArray<T> getSubRange(size_t first, size_t last){
			RW_ASSERT(first<last);

			return IndexedTriArray<T>(_objArr,_idxArr,_first+first, _first+last);
		}

		rw::common::Ptr<TriMesh> clone() const{
			return new IndexedTriArray<T>(_objArr,_idxArr,_first, _last);
		}

		size_t getGlobalIndex(int idx){ return _first+idx; }

		// **** inherited from trimesh
		//! @copydoc TriMesh::operator[]
		rw::geometry::Triangle<> operator[](size_t i) const {
			return _objArr->getTriangle( (*_idxArr)[i]);
		}

		//! @copydoc TriMesh::getTriangle
		rw::geometry::Triangle<> getTriangle(size_t idx) const{
			return (*this)[idx];
		}

		//! @copydoc TriMesh::getType
		rw::geometry::GeometryData::GeometryType getType() const{
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
