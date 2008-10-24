/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "GeometryBox.hpp"

using namespace rw::geometry;
using namespace rw::math;

namespace
{
	std::string toString(float dx, float dy, float dz){
		std::stringstream str;
		//str << "Box " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

GeometryBox::GeometryBox(float dx, float dy, float dz):
	Geometry( toString(dx,dy,dz) )
{
    float x = dx/2;
    float y = dy/2;
    float z = dz/2;

    Vector3D<float> p1(x, y, z);
    Vector3D<float> p2(x, y, -z);
    Vector3D<float> p3(-x, y, -z);
    Vector3D<float> p4(-x, y, z);

    Vector3D<float> p5(x, -y, z);
    Vector3D<float> p6(x, -y, -z);
    Vector3D<float> p7(-x, -y, -z);
    Vector3D<float> p8(-x, -y, z);

    _faces.push_back(Face<float>(p1, p2, p3));
    _faces.push_back(Face<float>(p3, p4, p1));

    _faces.push_back(Face<float>(p1, p5, p6));
    _faces.push_back(Face<float>(p6, p2, p1));

    _faces.push_back(Face<float>(p3, p2, p6));
    _faces.push_back(Face<float>(p6, p7, p3));

    _faces.push_back(Face<float>(p5, p8, p7));
    _faces.push_back(Face<float>(p7, p6, p5));

    _faces.push_back(Face<float>(p1, p4, p8));
    _faces.push_back(Face<float>(p8, p5, p1));

    _faces.push_back(Face<float>(p4, p3, p7));
    _faces.push_back(Face<float>(p7, p8, p4));
}

GeometryBox::~GeometryBox() {

}


const std::vector<Face<float> >& GeometryBox::getFaces() const {
    return _faces;
}
