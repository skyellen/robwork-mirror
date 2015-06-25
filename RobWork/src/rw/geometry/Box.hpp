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


#ifndef RW_GEOMETRY_BOXPRIMITIVE_HPP_
#define RW_GEOMETRY_BOXPRIMITIVE_HPP_

#include "Primitive.hpp"

#include <rw/common/Ptr.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{
	/**
	 * @brief a box primitive, origin is in center of box
	 */
	class Box: public Primitive {
	public:
		//! @brief smart pointer type to this class
        typedef rw::common::Ptr<Box> Ptr;
		/**
		 * @brief constructor - creates a 1x1x1 sided box
		 */
		Box():_dx(1),_dy(1),_dz(1){};

		/**
		 * @brief constructor
		 * @param x [in] width in x axis
		 * @param y [in] width in y axis
		 * @param z [in] width in z axis
		 */
		Box(double x, double y, double z);

		/**
		 * @brief constructor
		 * @param initQ [in] vector with (x,y,z)
		 */
		Box(const rw::math::Q& initQ);

		//! @brief destructor
		virtual ~Box();

		// inherited from Primitive

		//! @copydoc Primitive::createMesh
		TriMesh::Ptr createMesh(int resolution) const;

		//! @copydoc Primitive::getParameters
		virtual rw::math::Q getParameters() const;
		
		//! @copydoc Primitive::setParameters
		virtual void setParameters(const rw::math::Q& q);

		//! @copydoc GeometryData::getType
		GeometryType getType() const { return BoxPrim; };
	protected:
		 bool doIsInside(const rw::math::Vector3D<>& point);
	private:
		double _dx,_dy,_dz;
	};
	//! @}

#ifdef RW_USE_DEPRECATED
    /**
     * @brief Pointer to Box
     */
    typedef rw::common::Ptr<Box> BoxPtr;
#endif
} // geometry
} // rw

#endif /* BOX_HPP_ */
