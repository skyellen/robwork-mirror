#include "GeometryFactory.hpp"

#include "GeometryBox.hpp"
#include "GeometryCylinder.hpp"

#include <rw/common/macros.hpp>
#include <sstream>

using namespace rw::geometry;

namespace
{
    Geometry* constructBox(std::stringstream& sstr)
    {
        float x, y, z;
        sstr >> x;
        sstr >> y;
        sstr >> z;

        return new GeometryBox(x, y, z);
    }

    Geometry* constructCylinder(std::stringstream& sstr)
    {
        float radius, height;
        unsigned int divisions;
        sstr >> radius;
        sstr >> height;
        sstr >> divisions;

        return new GeometryCylinder(radius, height, divisions);
    }
}

Geometry* GeometryFactory::GetGeometry(const std::string& str)
{
    if (str.empty() || str[0] != '#')
        RW_THROW("String identifier of a geometric primitive must start with \"#\"");

    std::stringstream sstr(str);
    std::string type;
    sstr >> type;
    std::cout << "type = " << type << "\n";

    if (type == "#Box")
        return constructBox(sstr);
    if (type == "#Cylinder")
        return constructCylinder(sstr);
    else {
        RW_THROW("Unable to construct geometry from string: \"" << str << "\"");

        // To avoid a compiler warning.
        return 0;
    }
}
