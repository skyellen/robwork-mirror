#include "Contour2Dto3D.hpp"

#include "triangulate.hpp"
#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>


using namespace rw::math;
using namespace rw::geometry;

typedef Vector2D<double> P2D;
typedef TriangleN1<float> Face;
typedef Vector3D<float> P3D;

namespace
{
    // Faces for either the top or bottom of the geometry.
    std::vector<Face> contourFaces(
        const Contour2D& object,
        double z,
        const P3D& normal)
    {
        namespace r = ratcliff_triangulate;

		r::Vector2DVector pnts;
        BOOST_FOREACH(const ContourPoint& pnt, object.contour) {
            pnts.push_back(
                r::Vector2D(
                    (float)pnt.getPosition()(0),
                    (float)pnt.getPosition()(1)));
        }
        r::Vector2DVector triangulation;
        r::Triangulate::Process(pnts, triangulation);

        std::vector<Face> result;
        const int cnt = triangulation.size() / 3;

        for (int i = 0; i < cnt; i++) {
            const r::Vector2D& p1 = triangulation[i * 3 + 0];
            const r::Vector2D& p2 = triangulation[i * 3 + 1];
            const r::Vector2D& p3 = triangulation[i * 3 + 2];

            result.push_back(

                Face(
                    P3D((float)p1.GetX(), (float)p1.GetY(), (float)z),
                    P3D((float)p2.GetX(), (float)p2.GetY(), (float)z),
                    P3D((float)p3.GetX(), (float)p3.GetY(), (float)z),
                    normal));
        }
        return result;
    }

    P3D unit(const P3D& dir)
    {
        return dir / dir.norm2();
    }

    P3D to3D(const P2D& p, double z)
    {
        return P3D((float)p(0), (float)p(1), (float)z);
    }

    // The horizontal faces of the object.
    std::vector<Face> sideFaces(
        const Contour2D& object,
        double height)
    {
        std::vector<Face> result;
        const int len = object.contour.size();
        for (int i = 1; i < len; i++) {
            const P2D p = object.contour[i - 1].getPosition();
            const P2D q = object.contour[i].getPosition();

            const P3D a1 = to3D(p, 0);
            const P3D a2 = to3D(q, 0);
            const P3D b1 = to3D(p, height);
            const P3D b2 = to3D(q, height);

            const P3D normal = unit(cross(a2 - a1, b1 - a1));

            result.push_back(Face(a1, a2, b1, normal));
            result.push_back(Face(a2, b2, b1, normal));
        }
        return result;
    }
}

std::vector<Face> Contour2Dto3D::contourGeometry(
    const Contour2D& object,
    double height)
{
    if (object.contour.size() < 3) RW_THROW("No contour.");

    std::vector<Face> faces;

    const std::vector<Face> side = sideFaces(object, height);
    faces.insert(faces.end(), side.begin(), side.end());

    const std::vector<Face> floor = contourFaces(object, 0, P3D(0, 0, -1));
    faces.insert(faces.end(), floor.begin(), floor.end());

    const std::vector<Face> top = contourFaces(object, height, P3D(0, 0, 1));
    faces.insert(faces.end(), top.begin(), top.end());

    return faces;
}
