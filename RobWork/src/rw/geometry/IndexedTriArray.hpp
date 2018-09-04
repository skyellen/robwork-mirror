#ifndef RW_GEOMETRY_INDEXEDTRIARRAY_HPP_
#define RW_GEOMETRY_INDEXEDTRIARRAY_HPP_

#include <rw/geometry/TriMesh.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/tuple/tuple.hpp>

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
		rw::geometry::TriMesh::Ptr _objArr;
		rw::common::Ptr< std::vector<T> > _idxArr;
		rw::common::Ptr< std::vector< rw::math::Vector3D<float> > > _centerArr;
		rw::common::Ptr< std::vector< float > > _valArr;

		typedef boost::tuple<T,float,rw::math::Vector3D<float> > ValueType;
		rw::common::Ptr< std::vector<ValueType> > _valCenterArr;
		size_t _first,_last;

	public:

		/**
		 * @brief constructor - creates a proxy that initially maps triangles
		 * from [0:0,1:1,2:2,....,i:i]
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 */
		IndexedTriArray(TriMesh::Ptr objArr):
			_objArr(objArr),
			_idxArr(rw::common::ownedPtr(new std::vector<T>(objArr->getSize()))),
			_centerArr(rw::common::ownedPtr(new std::vector< rw::math::Vector3D<float> >(objArr->getSize()))),
			_valArr(rw::common::ownedPtr(new std::vector<float>(objArr->getSize()))),
			_valCenterArr( rw::common::ownedPtr( new std::vector<ValueType>( objArr->getSize() ) ) ),
			_first(0),
			_last(objArr->getSize())
		{
			for(size_t i=0;i<_idxArr->size();i++){
				(*_idxArr)[i] = (T)i;
			}
			// calculate triangle center points

			Triangle<float> tri;
            for(size_t i=0;i<_objArr->getSize();i++){
                objArr->getTriangle(i, tri);
                boost::tuples::get<0>( (*_valCenterArr)[i] ) = (int)i;
                boost::tuples::get<2>( (*_valCenterArr)[i] ) = ((tri[0]+tri[1]+tri[2])/3);
            }

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
		IndexedTriArray(TriMesh::Ptr objArr,
						rw::common::Ptr< std::vector<T> > idxArr):
			_objArr(objArr),
			_idxArr(idxArr),
			_centerArr(rw::common::ownedPtr(new std::vector< rw::math::Vector3D<float> >(objArr->getSize()))),
			_valArr(rw::common::ownedPtr(new std::vector<float>(objArr->getSize()))),
			_valCenterArr( rw::common::ownedPtr( new std::vector<ValueType>( objArr->getSize() ) ) ),
			_first(0),
			_last(idxArr->size())
		{
            // calculate triangle center points
            Triangle<float> tri;
            for(size_t i=0;i<_objArr->getSize();i++){
                objArr->getTriangle(i, tri);
                boost::tuples::get<0>( (*_valCenterArr)[i] ) = (int)i;
                boost::tuples::get<2>( (*_valCenterArr)[i] ) = ((tri[0]+tri[1]+tri[2])/3);
            }
		}

		/**
		 * @brief constructor - creates a proxy that only references part of
		 * the triangle mesh. the part is specified in the range from
		 * \b first to \b last
		 * @param objArr [in] the triangle mesh on which to create a proxy
		 * @param idxArr [in] the index mapping
		 * @param first [in] the first index
		 */
		IndexedTriArray(TriMesh::Ptr objArr,
						rw::common::Ptr< std::vector<T> > idxArr,
						rw::common::Ptr< std::vector< rw::math::Vector3D<float> > > centerArr,
						rw::common::Ptr< std::vector< float > > valArr,
						rw::common::Ptr< std::vector< boost::tuple<T,float,rw::math::Vector3D<float> > > > valCenterArr,
						size_t first,
						size_t last):
			_objArr(objArr),
			_idxArr(idxArr),
			_centerArr(centerArr),
			_valArr(valArr),
			_valCenterArr(valCenterArr),
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
				 const std::vector< rw::math::Vector3D<float> >& centerArr
				 ):
				_splitAxis(splitAxis),
				_t3d(t3d),
				_centers(centerArr)
			{
			    R_s0 = (float) _t3d.R()(splitAxis,0);
			    R_s1 = (float) _t3d.R()(splitAxis,1);
			    R_s2 = (float) _t3d.R()(splitAxis,2);
			    P_s = (float) _t3d.P()(splitAxis);
			}

			bool operator()(const size_t& i0, const size_t& i1) {
				using namespace rw::math;
				using namespace rw::geometry;
				//std::cout << i0 << "  -- --- - " << i1 << "\n";
/*
				RW_ASSERT(0<=i0 && i0<_mesh.getSize());
				RW_ASSERT(0<=i1 && i1<_mesh.getSize());
				const Triangle<> &tri = _mesh.getTriangle(i0);
				const Triangle<> &t1 = _mesh.getTriangle(i1);
				Vector3D<> c0 = _t3d* ((tri[0]+tri[1]+tri[2])/3);
				Vector3D<> c1 = _t3d* ((t1[0]+t1[1]+t1[2])/3);


*/
				// this basiacally takes one column of the rotation matrix and performs only the relevant multiplications in v = T*center
				const rw::math::Vector3D<float>& ic0 = _centers[i0];
                const rw::math::Vector3D<float>& ic1 = _centers[i1];
				float val1 = ic0[0]*R_s0 + ic0[1]*R_s1 + ic0[2]*R_s2 + P_s;
				float val2 = ic1[0]*R_s0 + ic1[1]*R_s1 + ic1[2]*R_s2 + P_s;
				return  val1<val2;

				//return i0<i1;
			}
			const int _splitAxis;
			const rw::math::Transform3D<> _t3d;
			const std::vector< rw::math::Vector3D<float> >& _centers;
			float R_s0,R_s1,R_s2,P_s;
		};

        struct TrisIdxSort2
        {
            TrisIdxSort2(const std::vector< float >& valArr ):
                _valArr(valArr)
            {
            }

            bool operator()(const size_t& i0, const size_t& i1) {
                using namespace rw::math;
                using namespace rw::geometry;
                //std::cout << i0 << "  -- --- - " << i1 << "\n";
                return  _valArr[i0]<_valArr[i1];
            }

            const std::vector< float >& _valArr;
        };


        struct IdxValSort
        {
            IdxValSort()
            {}

            bool operator()(const boost::tuple<T,float,rw::math::Vector3D<float> >& i0, const boost::tuple<T,float,rw::math::Vector3D<float> >& i1) {
                return boost::tuples::get<1>( i0 ) < boost::tuples::get<1>( i1 );
            }
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
		    using namespace rw::math;
		    rw::math::Transform3D<float> t3df = cast<float>(t3d);
		    float R_s0 = t3df.R()(0,axis);
            float R_s1 = t3df.R()(1,axis);
            float R_s2 = t3df.R()(2,axis);
            float P_s = t3df.P()(axis);

            // first transform the requested splitaxis values
            for(size_t i=_first;i<_first+size(); i++){
                boost::tuple<T,float,rw::math::Vector3D<float> >& elem = (*_valCenterArr)[i];
                const rw::math::Vector3D<float>& ic0 = boost::tuples::get<2>(elem);
                boost::tuples::get<1>(elem) =  ic0[0]*R_s0 + ic0[1]*R_s1 + ic0[2]*R_s2 + P_s; //(t3df*ic0)(axis);
            }

			//std::sort(_idxArr->begin(), _idxArr->end(), TrisIdxSort(axis, t3d, *_objArr));
	        /*if(size()<10){
	            std::cout << "PRE" << _first << ";" << _last << "\n-- ";
	            for(int i=0;i<size();i++)
	                std::cout << ", " << (*_idxArr)[i];
	            std::cout << "\n";
	        }*/
		    // TODO: should try sorting directly on the tri centers to get better speed/performance
		    //std::sort( &((*_idxArr)[_first]), &((*_idxArr)[_first])+ size(), TrisIdxSort2(axis, t3d, *_centerArr));
            //std::sort( &((*_idxArr)[_first]), &((*_idxArr)[_first])+ size(), TrisIdxSort2(*_valArr));
            std::sort( &((*_valCenterArr)[_first]), &((*_valCenterArr)[_first])+ size(), IdxValSort());

            /*if(size()<10){
                std::cout << "POST\n-- ";
                for(int i=0;i<size();i++)
                    std::cout << ", " << (*_idxArr)[i];
                std::cout << "\n";
            }*/
		}


		IndexedTriArray<T> getSubRange(size_t first, size_t last){
			RW_ASSERT(first<last);

			return IndexedTriArray<T>(_objArr,_idxArr,_centerArr, _valArr, _valCenterArr, _first+first, _first+last);
		}

		rw::common::Ptr<TriMesh> clone() const{
			return rw::common::ownedPtr( new IndexedTriArray<T>(_objArr,_idxArr,_centerArr, _valArr, _valCenterArr, _first, _last) );
		}

				//! @copydoc TriMesh::scale
		void scale(double s) {
			_objArr->scale(s);
			for (size_t i = 0; i<_centerArr->size(); i++) {
				_centerArr->at(i) *= static_cast<float>(s);
			}

		}

		//size_t getGlobalIndex(int idx){ return (*_idxArr)[_first+idx]; }
		inline size_t getGlobalIndex(int idx) const { return boost::tuples::get<0>( (*_valCenterArr)[_first+idx] ); }

		// **** inherited from trimesh
		//! @copydoc TriMesh::operator[]
		inline rw::geometry::Triangle<> operator[](size_t i) const {
			return _objArr->getTriangle( getGlobalIndex((int)i) );
		}

		//! @copydoc TriMesh::getTriangle
		inline rw::geometry::Triangle<> getTriangle(size_t idx) const{
			return (*this)[idx];
		}

        inline void getTriangle(size_t i, Triangle<double>& dst) const {
            _objArr->getTriangle( getGlobalIndex((int)i) , dst);
        }

        //! @copydoc TriMesh::getTriangle
        inline void getTriangle(size_t i, Triangle<float>& dst) const {
            _objArr->getTriangle( getGlobalIndex((int)i) , dst);
        }


		//! @copydoc TriMesh::getType
		rw::geometry::GeometryData::GeometryType getType() const{
			return GeometryData::UserType;
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
