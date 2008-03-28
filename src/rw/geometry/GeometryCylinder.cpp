#include "GeometryCylinder.hpp"

#include <rw/math/Constants.hpp>

using namespace rw::geometry;
using namespace rw::math;

namespace {
	std::string toString(float dx, float dy, unsigned int dz){
		std::stringstream str;
		str << "Cylinder " << dx << " " << dy << " " << dz;
		return str.str();
 	}
}

GeometryCylinder::GeometryCylinder(float radius, float height, unsigned int level):
	Geometry( toString(radius,height,level) )
{
    float z = height/2.0f;

    for (unsigned int i = 0; i < level; i++) {
        //Construct Triangles for curved surface
        float x1 = (float)(radius * cos(i * 2 * Pi/level));
        float y1 = (float)(radius * sin(i * 2 * Pi/level));

        float x2 = (float)(radius * cos((i+1) * 2 * Pi/level));
        float y2 = (float)(radius * sin((i+1) * 2 * Pi/level));

        Vector3D<float> p1(x1, y1, z);
        Vector3D<float> p2(x1, y1, -z);
        Vector3D<float> p3(x2, y2, z);
        Vector3D<float> p4(x2, y2, -z);
        
        _faces.push_back(Face<float>(p1, p2, p3));
        _faces.push_back(Face<float>(p2, p4, p3));

        //Construct triangles for the end-plates
        _faces.push_back(
            Face<float>(
                Vector3D<float>(x1, y1, z),
                Vector3D<float>(x2, y2, z),
                Vector3D<float>(0,0,z)));

        _faces.push_back(
            Face<float>(
                Vector3D<float>(0,0,-z),
                Vector3D<float>(x2, y2, -z),
                Vector3D<float>(x1, y1, -z)));
    }
}

GeometryCylinder::~GeometryCylinder()
{}

const std::list<Face<float> >& GeometryCylinder::getFaces() const
{
    return _faces;
}
