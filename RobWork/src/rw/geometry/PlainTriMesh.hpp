/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_GEOMETRY_PLAINTRIMESH_HPP_
#define RW_GEOMETRY_PLAINTRIMESH_HPP_

#include "TriMesh.hpp"

//#include <boost/type_traits.hpp>


namespace rw {
namespace geometry {
    //! @addtogroup geometry
    // @{
    //! @file PlainTriMesh.hpp

	/**
	 * @brief a triangle mesh representation that maintains a list of simple triangles.
	 *
	 * This class is templated and can be setup with different types of triangle storage.
	 * Mainly this concerns single or double precision but also number of normals in each Triangle.
	 * Check out Triangle.hpp to get an idea of the different types.
	 *
	 * The PlainTriMesh can be used as follows
	 * \code
	 * // create trimesh
	 * PlainTriMesh<TriangleN1<float> > mesh;
	 * // add data
	 * mesh.add( TriangleN1<float>(v1,v2,v3) );
     * mesh.add( TriangleN1<float>(v1,v2,v3) );
     * mesh.add( TriangleN1<float>(v1,v2,v3) );
     *  // and access the mesh
     * TriangleN1<float> tri_index1 = mesh[1];
     * Vector3D<float> normal = mesh[2].getFaceNormal();
	 * \endcode
	 *
	 * To convert the plain trimesh to a more efficient mesh representation take a look at
	 * TriangleUtil::toIndexedTriMesh().
	 *
	 */
	template <class TRI = Triangle<> >
	class PlainTriMesh: public TriMesh/*<typename TRI::value_type>*/ {
	private:
		std::vector<TRI> _triangles;

	public:
		//! smart pointer type of this class
		typedef rw::common::Ptr<PlainTriMesh<TRI> > Ptr;

	    //! the triangle type
	    typedef typename TRI::value_type value_type;
		/**
		 * @brief constructor
		 */
		PlainTriMesh(int initSize=0):
		    _triangles(initSize)
		{}

		/**
		 * @brief constructor
		 */
		PlainTriMesh(const std::vector<TRI>& tris):
			_triangles(tris)
		{}

		virtual ~PlainTriMesh(){}


		//! @copydoc TriMesh::scale
		void scale(double scale) {			
			for (typename std::vector<TRI>::iterator it = _triangles.begin(); it != _triangles.end(); ++it) {
				for (size_t i = 0; i<3; i++) {
					(*it).getVertex(i) *= static_cast<value_type>(scale);
				}
			}
		}


		/**
		 * @brief add a triangle to the triangle mesh.
		 * @param triangle [in] Triangle to add. The triangle is copied.
		 */
		void add(const TRI& triangle){
			_triangles.push_back(triangle);
		}

		/**
		 * @brief Add all triangles in the mesh \b triangles to this
		 * @param [in] Triangle mesh for which to add triangles
		 */
		void add(typename PlainTriMesh<TRI>::Ptr triangles) {
			_triangles.insert(_triangles.end(), triangles->_triangles.begin(), triangles->_triangles.end());
		}




		/**
		 * @brief Clears the list of triangles
		 */
		void clear() {
			_triangles.clear();
		}
		/**
		 * @brief the vector of triangles
		 * @return a reference to the triangle vector
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

		/**
		 * @brief resize the triangle mesh
		 * @param i [in] new size of mesh.
		 */
		void resize(size_t i){
		    _triangles.resize(i);
		}

		// Inherited from TriMesh
		//! @copydoc TriMesh::getTriangle
		Triangle<double> getTriangle(size_t idx) const {
		    using namespace rw::math;

		    if( ::boost::is_same<float, value_type>::value ){
		        const TRI &triA = _triangles[idx];
		        Triangle<double> tri;
		        tri[0] = cast<double>( triA[0] );
		        tri[1] = cast<double>( triA[1] );
		        tri[2] = cast<double>( triA[2] );
                return tri;
		    } else {
		        // we need to force the cast since the compiler does not recognize is_float
		        return *(reinterpret_cast<const Triangle<double>* >(&_triangles[idx].getTriangle()));
		    }
		}

        //! @copydoc TriMesh::getTriangle
        void getTriangle(size_t idx, Triangle<double>& dst) const {
            using namespace rw::math;
            if( ::boost::is_same<float, value_type>::value ){
                const TRI &triA = _triangles[idx];
                dst[0] = cast<double>( triA[0] );
                dst[1] = cast<double>( triA[1] );
                dst[2] = cast<double>( triA[2] );
            } else {
                dst = *(reinterpret_cast<const Triangle<double>* >(&_triangles[idx].getTriangle()));
            }
        }

        //! @copydoc TriMesh::getTriangle
        void getTriangle(size_t idx, Triangle<float>& dst) const {
            using namespace rw::math;
            if( ::boost::is_same<float, value_type>::value ){
                dst = *(reinterpret_cast<const Triangle<float>* >(&_triangles[idx].getTriangle()));
            } else {
                const TRI &triA = _triangles[idx];
                dst[0] = cast<float>( triA[0] );
                dst[1] = cast<float>( triA[1] );
                dst[2] = cast<float>( triA[2] );
            }
        }

		/**
		 * @copydoc TriMesh::getSize
		 */
		size_t getSize() const {
			return _triangles.size();
		}

		/**
		 * @copydoc TriMesh::size
		 */
		size_t size() const {
			return _triangles.size();
		}

		//! @copydoc GeometryData::getType
		GeometryData::GeometryType getType() const{
		    return GeometryData::PlainTriMesh;
		};

		//! @copydoc TriMesh::clone
		rw::common::Ptr<TriMesh> clone() const{
			return rw::common::ownedPtr( new PlainTriMesh(*this) );
		}


	};

	//! @brief tri mesh, double, no normals
	typedef PlainTriMesh<Triangle<double> > PlainTriMeshD;
	//! @brief tri mesh, float, no normals
	typedef PlainTriMesh<Triangle<float> > PlainTriMeshF;
	//! @brief tri mesh, double, 1 face normal
	typedef PlainTriMesh<TriangleN1<double> > PlainTriMeshN1D;
	//! @brief tri mesh, float, 1 face normal
	typedef PlainTriMesh<TriangleN1<float> > PlainTriMeshN1F;
	//! @brief tri mesh, double, 3 vertex normals
	typedef PlainTriMesh<TriangleN3<double> > PlainTriMeshN3D;
	//! @brief tri mesh, float, 3 vertex normals
	typedef PlainTriMesh<TriangleN3<float> > PlainTriMeshN3F;
	//! @}

} // geometry
} // rw

#endif /*TRIMESH_HPP_*/
