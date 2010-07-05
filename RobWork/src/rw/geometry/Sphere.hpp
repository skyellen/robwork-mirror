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


#ifndef RW_GEOMETRY_SPHERE_HPP_
#define RW_GEOMETRY_SPHERE_HPP_

//! @file rw/geometry/Sphere.hpp

#include "Primitive.hpp"

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief a sphere primitive. centr in (0,0,0) and a radius.
	 */
	class Sphere: public Primitive {
	public:
		//! constructor
		Sphere(const rw::math::Q& initQ):_radius(initQ(0)){};

		//! @brief constructor
		Sphere(double radi):_radius(radi){};

		//! @brief destructor
		virtual ~Sphere() {}

		// inherited from Primitive
		//! @copydoc Primitive::createMesh
		TriMeshPtr createMesh(int resolution) const;

		//! @copydoc Primitive::getParameters
		rw::math::Q getParameters() const{ return rw::math::Q(1,_radius);};

		//! @copydoc GeometryData::getType
		GeometryData::GeometryType getType() const { return GeometryData::SpherePrim; };

	private:
		double _radius;
	};
	//! @}

} // geometry
} // rw


#endif /* SPHERE_HPP_ */
