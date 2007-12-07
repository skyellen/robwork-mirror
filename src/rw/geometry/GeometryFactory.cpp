#include "GeometryFactory.hpp"

#include "GeometryBox.hpp"
#include "GeometryCylinder.hpp"

#include <rw/common/macros.hpp>
#include <sstream>

using namespace rw::geometry;

namespace {
    
    Geometry* constructBox(std::stringstream& sstr) {
        float x, y, z;
        sstr >> x;
        sstr >> y;
        sstr >> z;
        
        return new GeometryBox(x, y, z);
    }

    Geometry* constructCylinder(std::stringstream& sstr) {
        float radius, height;
        unsigned int divisions;
        sstr >> radius;
        sstr >> height;
        sstr >> divisions;
        return new GeometryCylinder(radius, height, divisions);
    }




}

Geometry* GeometryFactory::GetGeometry(const std::string& str) {
    if (str[0] != '#') 
        RW_THROW("String identifier of a geometric primitive must start with \"#\" ");

    try {


        
        std::stringstream sstr(str);
        std::string type;
        sstr >> type;
        std::cout<<"type = "<<type<<std::endl;
        
        
        if (type == "#Box")
            return constructBox(sstr);
        if (type == "#Cylinder")
            return constructCylinder(sstr);
        
    } catch (...) {}
    RW_THROW("Unable to construct geometry from string: \""<<str<<"\"");

}
