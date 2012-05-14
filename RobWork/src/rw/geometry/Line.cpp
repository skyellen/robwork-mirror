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

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>

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


std::vector<Line> Line::makeGrid(int dim_x, int dim_y, double size_x, double size_y, const rw::math::Vector3D<>& xdir, const rw::math::Vector3D<>& ydir){
    //Rotation3D<> rot =< EAA<>(Vector3D<>::z(), normal).toRotation3D();
    rw::math::Vector3D<> xdir_n = normalize(xdir);
    rw::math::Vector3D<> ydir_n = normalize(ydir);
    std::vector<Line> lines;
    double halfsize_x = size_x*dim_x/2.0;
    double halfsize_y = size_y*dim_y/2.0;
    for(int dx=0;dx<=dim_x;dx++){
       Vector3D<> p1 = xdir_n*(dx*size_x-halfsize_x) + ydir_n*halfsize_y;
       Vector3D<> p2 = xdir_n*(dx*size_x-halfsize_x) + ydir_n* -halfsize_y;
       lines.push_back(Line(p1, p2));
    }
    for(int dy=0;dy<=dim_y;dy++){
        Vector3D<> p1 = xdir_n*halfsize_x + ydir_n*(dy*size_y-halfsize_y);
        Vector3D<> p2 = xdir_n*-halfsize_x + ydir_n*(dy*size_y-halfsize_y);
        lines.push_back(Line(p1,p2));
    }
    return lines;
}
