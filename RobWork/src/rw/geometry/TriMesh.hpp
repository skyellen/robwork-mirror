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

#ifndef RW_GEOMETRY_TRIMESH_HPP_
#define RW_GEOMETRY_TRIMESH_HPP_

#include <rw/common/Ptr.hpp>

#include "GeometryData.hpp"
#include "Triangle.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief interface of a triangle mesh. The interface defines a way to get
	 * triangles from a triangle array/mesh.
	 */
	class TriMesh: public GeometryData {
	protected:
		TriMesh():_isConvex(false){}

	public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<TriMesh> Ptr;

		/**
		 * @brief destructor
		 */
		virtual ~TriMesh(){};

		/**
		 * @brief gets the triangle at index idx.
		 */
		virtual Triangle<double> getTriangle(size_t idx) const = 0;


		virtual void getTriangle(size_t idx, Triangle<double>& dst) const = 0;
        /**
         * @brief gets the triangle at index idx, but with floating precision
         */
		virtual void getTriangle(size_t idx, Triangle<float>& dst) const = 0;


		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t getSize() const = 0;

		/**
		 * @brief gets the number of triangles in the triangle array.
		 */
		virtual size_t size() const = 0;

		/**
		 * @brief make a clone of this triangle mesh
		 * @return clone of this trimesh
		 */
		virtual rw::common::Ptr<TriMesh> clone() const = 0;

		/**
		 * @brief Scale all vertices in the mesh.
		 */
		virtual void scale(double scale) = 0;

		//! @copydoc GeometryData::getTriMesh
		rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);

		//! @copydoc getTriMesh
		rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;

        //! @copydoc GeometryData::isConvex
        virtual bool isConvex() { return _isConvex; }

        void setConvexEnabled(bool isConvex){ _isConvex = isConvex; }
        
        /**
         * @brief calculate a volume of this triangle mesh
         */
        double getVolume() const;

	    /**
	     * @brief struct for iterating over the centers of triangles in a mesh
	     */
	    struct TriCenterIterator {
	        rw::math::Vector3D<> _pos;
	        const rw::geometry::TriMesh& _mesh;
	        size_t _first,_end;
	        bool _useAreaWeight;
	        TriCenterIterator(const rw::geometry::TriMesh& mesh, bool useAreaWeight=false):
	            _mesh(mesh),_first(0),_end(mesh.getSize()),_useAreaWeight(useAreaWeight)
	        {}

	        rw::math::Vector3D<>& operator*() {
	            return _pos;
	        }

	        rw::math::Vector3D<>* operator->() { return &_pos; }

	        TriCenterIterator& operator++(){ inc(); return *this; }

	        bool operator==(const TriCenterIterator& other) const{ return _first == other._end;}
	        bool operator!=(const TriCenterIterator& other) const { return _first < other._end;}

	        void inc(){
	            using namespace rw::geometry;
	            using namespace rw::math;
	            using namespace rw::common;

	            ++_first;
	            if(_first!=_end){
	                Triangle<> tri = _mesh.getTriangle(_first);
	                if(_useAreaWeight){
	                    double area = tri.calcArea();
	                    _pos = area*(tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
	                } else {
	                    _pos = (tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0;
	                }
	            }
	        }
	    };

        /**
         * @brief struct for iterating over the centers of triangles in a mesh
         */
        struct VerticeIterator {
            rw::math::Vector3D<> _pos;
            const rw::geometry::TriMesh& _mesh;

            size_t _first,_end,_subIdx;

            VerticeIterator(const rw::geometry::TriMesh& mesh):
                _mesh(mesh),_first(0),_end(mesh.getSize()),_subIdx(0)
            {}

            rw::math::Vector3D<>& operator*() {
                return _pos;
            }

            rw::math::Vector3D<>* operator->() { return &_pos; }

            VerticeIterator& operator++(){ inc(); return *this; }

            bool operator==(const VerticeIterator& other) const{ return _first == other._end;}
            bool operator!=(const VerticeIterator& other) const { return _first < other._end;}

            void inc(){
                using namespace rw::geometry;
                using namespace rw::math;
                using namespace rw::common;


                if(_first!=_end){
                    _pos = _mesh.getTriangle(_first).getVertex(_subIdx);
                    _subIdx++;
                    if(_subIdx==3){
                        ++_first;
                        _subIdx=0;
                    }
                }
            }
        };

	private:
        bool _isConvex;

	};
	//! @}
} // geometry
} // rw



#endif /*TRIMESH_HPP_*/
