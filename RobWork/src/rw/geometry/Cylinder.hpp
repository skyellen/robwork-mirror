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


#ifndef RW_GEOMETRY_CYLINDER_HPP_
#define RW_GEOMETRY_CYLINDER_HPP_

#include "Primitive.hpp"

namespace rw {
namespace geometry {
//! @addtogroup geometry
// @{

	/**
	 * @brief a cylinder primitive. radius is in x-y plane and height is in z-axis
	 */
	class Cylinder: public Primitive {
	public:

		/**
		 * @brief constructor
		 */
		Cylinder();

		/**
		 * @brief Constructs cylinder primitive with the specified setup
		 *
		 * The cylinder is aligned with the height in the z-direction.
		 *
		 * @param radius [in] radius of the cylinder.
		 * @param height [in] height of the cylinder.
		 */
		Cylinder(float radius, float height);

		//! @brief destructor
		virtual ~Cylinder();

		double getRadius() const { return _radius;}

		double getHeight() const { return _height;}
		// inherited from Primitive

		//! @copydoc Primitive::createMesh
		TriMesh::Ptr createMesh(int resolution) const;

		//! @copydoc Primitive::getParameters
		rw::math::Q getParameters() const;

		//! @copydoc GeometryData::getType
		GeometryType getType() const { return CylinderPrim; };

	private:
		const float _radius;
		const float _height;
	};
	//! @}

} // geometry
} // rw

#endif /* CYLINDER_HPP_ */
