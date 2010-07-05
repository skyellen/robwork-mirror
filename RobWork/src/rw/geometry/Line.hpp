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


#ifndef RW_GEOMETRY_LINE_HPP_
#define RW_GEOMETRY_LINE_HPP_


#include "Primitive.hpp"

namespace rw {
namespace geometry {

	//! @addtogroup geometry
	// @{
	/**
	 * @brief a line segment in 3d. described by two points
	 */
	class Line: public Primitive {
	public:
		Line(const rw::math::Q& params);
		Line(const rw::math::Vector3D<>& p1, const rw::math::Vector3D<>& p2);
		virtual ~Line();

		inline rw::math::Vector3D<>& p1(){ return _p1;};
		inline const rw::math::Vector3D<>& p1() const{ return _p1;};
		inline rw::math::Vector3D<>& p2(){ return _p2;};
		inline const rw::math::Vector3D<>& p2() const { return _p2;};

		// inherited from Primitive
		TriMeshPtr createMesh(int resolution) const { return NULL;};

		rw::math::Q getParameters() const{ return rw::math::Q(2);};

		GeometryType getType() const { return LinePrim; };

	private:
		rw::math::Vector3D<> _p1,_p2;
		//rw::math::Q _param;
	};
	// @}
} // geometry
} // rw


#endif /* LINE_HPP_ */
