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

#include "Line.hpp"

using namespace rw::geometry;
using namespace rw::math;

Line::Line(const rw::math::Q& p){
	RW_ASSERT(p.size()==6);
	_p1 = Vector3D<>(p[0],p[1],p[2]);
	_p2 = Vector3D<>(p[3],p[4],p[5]);
}

Line::Line(const rw::math::Vector3D<>& p1, const rw::math::Vector3D<>& p2):
		_p1(p1),_p2(p2)
{

}

Line::~Line(){}
