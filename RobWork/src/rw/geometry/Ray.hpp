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


#ifndef RW_GEOMETRY_RAY_HPP
#define RW_GEOMETRY_RAY_HPP


#include "Primitive.hpp"

namespace rw {
namespace geometry {

/**
 * @brief a line sigment in 3d. described by two points
 */
class Ray {
public:
	Ray(rw::math::Vector3D<>& pos, rw::math::Vector3D<>& dir);
	virtual ~Ray();

	rw::math::Vector3D<>& pos();

	rw::math::Vector3D<>& dir();

private:
	rw::math::Vector3D<> _p1,_p2;
	//rw::math::Q _param;
};

} // geometry
} // rw


#endif /* RW_GEOMETRY_RAY_HPP*/
