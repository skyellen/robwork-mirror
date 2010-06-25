/**
   Construction of object geometries.
*/
#ifndef CONTOUR2DTO3D_HPP
#define CONTOUR2DTO3D_HPP

#include "Contour2D.hpp"

#include "Triangle.hpp"

namespace rw{
namespace geometry{

class Contour2Dto3D {
public:

    /**
       A 3D geometry for a 2D contour of height \b height.

       2D points (x,y) end up as (x, y, 0) in 3D.
    */
    static std::vector<rw::geometry::TriangleN1<float> > contourGeometry(
        const Contour2D& contour,
        double height);

};

}
}

#endif
