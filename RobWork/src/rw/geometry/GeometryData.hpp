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


#ifndef RW_GEOMETRY_GEOMETRYDATA_HPP_
#define RW_GEOMETRY_GEOMETRYDATA_HPP_

#include <rw/common/Ptr.hpp>

#include <string>
//! @file GeometryData.hpp

namespace rw { namespace geometry {
//! @addtogroup geometry
// @{


	class TriMesh;

	/**
	 * @brief an interface for geometry data.
	 */
	class GeometryData {
	public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<GeometryData> Ptr;

		//! @brief geometry data types
		typedef enum {PointCloud,
		              LineMesh,
		              PlainTriMesh,
					  IdxTriMesh,
					  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
					  LinePrim, PointPrim, PyramidPrim, ConePrim,
					  TrianglePrim, CylinderPrim, TubePrim, PlanePrim, RayPrim,
					  Quadratic,
					  UserType} GeometryType;
	    //! @brief Destructor
		virtual ~GeometryData(){}

		/**
		 * @brief the type of this primitive
		 */
		virtual GeometryType getType() const = 0;

		/**
		 * @brief gets a trimesh representation of this geometry data.
		 *
		 * The trimesh that is returned is by default a copy, which means
		 * ownership is transfered to the caller. Specifying \b forceCopy to false
		 * will enable copy by reference and ownership is not necesarilly transfered.
		 * This is more efficient, though pointer is only alive as long as this
		 * GeometryData is alive.
		 *
		 * @return TriMesh representation of this GeometryData
		 */
		virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) = 0;

		/**
		 * @brief test if this geometry data is convex
		 * @return
		 */
		virtual bool isConvex(){ return false;}

		/**
		 * @brief format GeometryType to string
		 * @param type
		 */
		static std::string toString(GeometryType type);
	};
	//! @}
}
}
#endif /* GEOMETRYDATA_HPP_ */
