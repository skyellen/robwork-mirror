#ifndef RW_GEOMETRY_TRIANGULATEPOLYGON_HPP
#define RW_GEOMETRY_TRIANGULATEPOLYGON_HPP

#include <vector>  // Include STL vector class.

#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include "PlainTriMesh.hpp"

namespace rw {
namespace geometry {

    /**
     * @brief class for triangulating a polygon
     */
    class Triangulate
    {
    public:
        /**
         * @brief triangulates a polygon or contuor described by a list of points
         * @param points [in] points must be a valid polygon/contour
         * @return a triangle mesh of triangles
         */
        //static PlainTriMeshN0D triangulatePolygon(const std::vector< rw::math::Vector3D<> >& points)

        /**
         * @brief
         * @param points
         * @return
         */
        static bool processPoints(const std::vector<rw::math::Vector2D<> >& points, std::vector<int>& result);

        // compute area of a contour/polygon
        static double calcArea(const std::vector<rw::math::Vector2D<> > &contour);

        // decide if point Px/Py is inside triangle defined by
        // (Ax,Ay) (Bx,By) (Cx,Cy)
        static bool insideTriangle2D(
            float Ax, float Ay,
            float Bx, float By,
            float Cx, float Cy,
            float Px, float Py);

    private:
        static bool snip(
            const std::vector<rw::math::Vector2D<> > &contour,
            int u,
            int v,
            int w,
            int n,
            int *V);
    };
}
}

#endif // end include guard
