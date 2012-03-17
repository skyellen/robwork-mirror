#ifndef RW_GEOMETRY_INDEXEDTRIARRAY_HPP_
#define RW_GEOMETRY_INDEXEDTRIARRAY_HPP_

#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace proximity {

	/**
	 * @brief this class is a proxy to a triangle mesh where the triangles
	 * can be indirectly copied in an efficient manner.
	 *
	 * Each "virtual" triangle index is mapped to the actual triangle index.
	 * By rearanging the mapping the order of the triangles in the mesh are
	 * rearanged to the proxy user, without changing the actual triangle mesh.
	 *
	 */
	template<class PRIM, class T=size_t>
	class IndexedPrimArray : public PrimArrayAccessor<PRIM> {
	private:
		rw::geometry::TriMesh::Ptr _objArr;
		rw::common::Ptr< std::vector<T> > _idxArr;
		size_t _first,_last;

	public:

		/**
		 * @brief constructor - creates a proxy that initially maps triangles
		 * from [0:0,1:1,2:2,....,i:i]
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 */
		IndexedPrimArray(rw::common::Ptr<PrimArrayAccessor<PRIM> > objArr):
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
		IndexedPrimArray(rw::common::Ptr<PrimArrayAccessor<PRIM> > objArr,
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
		IndexedPrimArray(rw::common::Ptr<PrimArrayAccessor<PRIM> > objArr,
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
		virtual ~IndexedPrimArray(){}

		//! @brief get the index mapping
		const std::vector<T>& getIndexes() const{
			return *_idxArr;
		}

	private:
		struct PrimIdxSort
		{
			PrimIdxSort(
				 const int splitAxis,
				 const rw::math::Transform3D<>& t3d,
				 const PrimArrayAccessor<PRIM>& mesh):
				_splitAxis(splitAxis),
				_t3d(t3d),
				_mesh(mesh)
			{}

			bool operator()(const size_t& i0, const size_t& i1) {
				using namespace rw::math;
				using namespace rw::geometry;
				//std::cout << i0 << "  -- --- - " << i1 << "\n";
				RW_ASSERT(0<=i0 && i0<_mesh.getSize());
				RW_ASSERT(0<=i1 && i1<_mesh.getSize());
				PRIM p1,p2;
				_mesh.getPrimitive(i0, p1);
				Vector3D<> c0 = p1.calcCenter();

				_mesh.getPrimitive(i1, p2);
				Vector3D<PRIM::value_type> c1 = _t3d*p2.calcCenter(); //((t1[0]+t1[1]+t1[2])/3.0);

				return  c0(_splitAxis)<c1(_splitAxis);
			}
			const int _splitAxis;
			const rw::math::Transform3D<> _t3d;
			const PrimArrayAccessor<PRIM>& _mesh;
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
			//std::sort(_idxArr->begin(), _idxArr->end(), TrisIdxSort(axis, t3d, *_objArr));
	        /*if(size()<10){
	            std::cout << "PRE" << _first << ";" << _last << "\n-- ";
	            for(int i=0;i<size();i++)
	                std::cout << ", " << (*_idxArr)[i];
	            std::cout << "\n";
	        }*/
		    std::sort( &((*_idxArr)[_first]), &((*_idxArr)[_first])+ size(), PrimIdxSort(axis, t3d, *_objArr));
            /*if(size()<10){
                std::cout << "POST\n-- ";
                for(int i=0;i<size();i++)
                    std::cout << ", " << (*_idxArr)[i];
                std::cout << "\n";
            }*/
		}


		IndexedPrimArray<T> getSubRange(size_t first, size_t last){
			RW_ASSERT(first<last);

			return IndexedPrimArray<T>(_objArr,_idxArr,_first+first, _first+last);
		}

		rw::common::Ptr<TriMesh> clone() const{
			return rw::common::ownedPtr( new IndexedTriArray<T>(_objArr,_idxArr,_first, _last) );
		}

		size_t getGlobalIndex(int idx){ return (*_idxArr)[_first+idx]; }

		// **** inherited from trimesh
		//! @copydoc TriMesh::operator[]
		PRIM operator[](size_t i) const {
			return _objArr->getPrimitive( (*_idxArr)[_first + i] );
		}

		//! @copydoc TriMesh::getTriangle
		PRIM getPrimitive(size_t idx) const{
			return (*this)[idx];
		}

        void getPrimitive(size_t i, PRIM& dst) const {
            _objArr->getPrimitive((*_idxArr)[_first + i], dst);
        }

		//! @copydoc TriMesh::getSize
		size_t getSize() const {
			return _last-_first;
		}

        size_t size() const {
            return _last-_first;
        }


	};

}
}

#endif /*INDEXEDARRAY_HPP_*/
