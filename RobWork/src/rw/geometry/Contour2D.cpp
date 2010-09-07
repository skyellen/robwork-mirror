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
#include "Contour2D.hpp"

using namespace rw::geometry;

double Contour2D::calcArea()
{
    int n = _points.size();

    double area=0.0;

    for(int p=n-1,q=0; q<n; p=q++)
    {
        area += _points[p].P()(0)*_points[q].P()(1) - _points[q].P()(0)*_points[p].P()(1);
    }
    return area*0.5f;
}


void Contour2D::write(Contour2D& objC, std::string file){
    std::ofstream ostr(file.c_str());
    if (!ostr.is_open())
        RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));

    ostr << "ObjectContour \n";
    ostr << "Size " << objC.size() << "\n";
    ostr << "Center " << objC.center()(0) << " " << objC.center()(1) << "\n";
    //std::cout << "size: " << objC.size() << std::endl;
    //std::cout << "center: " << objC.center << std::endl;
    for(size_t i=0; i<objC.size(); i++){
        Contour2D::Point &point = objC[i];
        rw::math::Vector2D<> pos = point.P();
        rw::math::Vector2D<> dir = point.N();
        ostr << "Pos " << pos(0) << " " << pos(1) << "\n";
        ostr << "Dir " << dir(0) << " " << dir(1) << "\n";
    }
    ostr.close();
}

Contour2D Contour2D::read(std::string file){
    std::ifstream inp(file.c_str());
    if (!inp.is_open())
        RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));
    std::string strToken;
    inp >> strToken;

    int size = 0;
    rw::math::Vector2D<> center;
    inp >> strToken >> size;
    inp >> strToken >> center(0) >> center(1);
    //std::cout << "size: " << size << std::endl;
    Contour2D objC;
    objC.center() = center;
    //std::cout << "center: " << center<< std::endl;
    objC.points().resize(size);
    for(size_t i=0; i<objC.size(); i++){
        rw::math::Vector2D<> pos, dir;

        inp >> strToken >> pos(0) >> pos(1);
        inp >> strToken >> dir(0) >> dir(1);
        objC[i] = Contour2D::Point(pos,dir);
    }
    inp.close();
    return objC;
}
