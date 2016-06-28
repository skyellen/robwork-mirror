
#ifndef RW_GEOMETRY_TriTriIntersectDeviller_HPP_
#define RW_GEOMETRY_TriTriIntersectDeviller_HPP_

#include <rw/geometry/Triangle.hpp>
#include <rw/math/Transform3D.hpp>
//#include <PQP/PQP.h>
namespace rw {
namespace geometry {

    /**
     * @brief tests if two triangles are intersecting using devillers method.
     *  - very robust triangle intersection test
     *  - uses no divisions
     *  - works on coplanar triangles
     *
     */
    template<class T=double>
    class TriTriIntersectDeviller {
    public:
        typedef T value_type;

        bool inCollision(const rw::math::Vector3D<T>& P1,
                         const rw::math::Vector3D<T>& P2,
                         const rw::math::Vector3D<T>& P3,
                         const rw::math::Vector3D<T>& Q1,
                         const rw::math::Vector3D<T>& Q2,
                         const rw::math::Vector3D<T>& Q3);

        bool inCollision(const rw::geometry::Triangle<T>& P,
                         const rw::geometry::Triangle<T>& Q);

        bool inCollision(const rw::geometry::Triangle<T>& P,
                         const rw::geometry::Triangle<T>& Q,
                         const rw::math::Transform3D<T>& PTQ);

    private:
        inline T mmax(T a, T b, T c)
        {
            T t = a;
            if (b > t) t = b;
            if (c > t) t = c;
            return t;
        }

        inline T mmin(T a, T b, T c)
        {
            T t = a;
            if (b < t) t = b;
            if (c < t) t = c;
            return t;
        }

        /**
         * @brief calculates the triple scalar product
         * @return
         */
        inline
        int project6(const rw::math::Vector3D<T>& ax,
                     const rw::math::Vector3D<T>& p1,
                     const rw::math::Vector3D<T>& p2,
                     const rw::math::Vector3D<T>& p3,
                     const rw::math::Vector3D<T>& q1,
                     const rw::math::Vector3D<T>& q2,
                     const rw::math::Vector3D<T>& q3)

        {
            T P1 = dot(ax, p1);
            T P2 = dot(ax, p2);
            T P3 = dot(ax, p3);
            T Q1 = dot(ax, q1);
            T Q2 = dot(ax, q2);
            T Q3 = dot(ax, q3);

            T mx1 = mmax(P1, P2, P3);
            T mn1 = mmin(P1, P2, P3);
            T mx2 = mmax(Q1, Q2, Q3);
            T mn2 = mmin(Q1, Q2, Q3);

            if (mn1 > mx2) return 0;
            if (mn2 > mx1) return 0;
            return 1;
        }




    };

    template<class T>
    bool TriTriIntersectDeviller<T>::inCollision(const rw::geometry::Triangle<value_type>& P,
                                     const rw::geometry::Triangle<value_type>& Q)
    {
        return inCollision(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
    }

    template<class T>
    bool TriTriIntersectDeviller<T>::inCollision(const rw::geometry::Triangle<value_type>& P,
                                     const rw::geometry::Triangle<value_type>& Q,
                                     const rw::math::Transform3D<value_type>& PTQ)
    {
        return inCollision(P[0], P[1], P[2], PTQ*Q[0], PTQ*Q[1], PTQ*Q[2]);
    }

    template<class T>
    bool TriTriIntersectDeviller<T>::inCollision(const rw::math::Vector3D<T>& P1,
                                     const rw::math::Vector3D<T>& P2,
                                     const rw::math::Vector3D<T>& P3,
                                     const rw::math::Vector3D<T>& Q1,
                                     const rw::math::Vector3D<T>& Q2,
                                     const rw::math::Vector3D<T>& Q3)
    {
            using namespace rw::math;

        /*return PQP::TriContact((PQP::PQP_REAL*)&P1[0],
                               (PQP::PQP_REAL*)&P2[0],
                               (PQP::PQP_REAL*)&P3[0],
                               (PQP::PQP_REAL*)&Q1[0],
                               (PQP::PQP_REAL*)&Q2[0],
                               (PQP::PQP_REAL*)&Q3[0]);
*/

        // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
        // Edges are (e1,e2,e3) and (f1,f2,f3).
        // Normals are n1 and m1
        // Outwards are (g1,g2,g3) and (h1,h2,h3).
        //
        // We assume that the triangle vertices are in the same coordinate system.
        //
        // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

        Vector3D<T> p1, p2, p3;
        Vector3D<T> q1, q2, q3;
        Vector3D<T> e1, e2, e3;
        Vector3D<T> f1, f2, f3;
        Vector3D<T> g1, g2, g3;
        Vector3D<T> h1, h2, h3;
        Vector3D<T> n1, m1;

        Vector3D<T> ef11, ef12, ef13;
        Vector3D<T> ef21, ef22, ef23;
        Vector3D<T> ef31, ef32, ef33;

        p1 = P1 - P1;
        p2 = P2 - P1;
        p3 = P3 - P1;

        q1 = Q1 - P1;
        q2 = Q2 - P1;
        q3 = Q3 - P1;

        e1 = p2 - p1;
        e2 = p3 - p2;
        e3 = p1 - p3;

        f1 = q2 - q1;
        f2 = q3 - q2;
        f3 = q1 - q3;

        cross(e1, e2, n1);
        cross(f1, f2, m1);

        cross(e1, n1, g1);
        cross(e2, n1, g2);
        cross(e3, n1, g3);
        cross(f1, m1, h1);
        cross(f2, m1, h2);
        cross(f3, m1, h3);

        cross(e1, f1, ef11);
        cross(e1, f2, ef12);
        cross(e1, f3, ef13);
        cross(e2, f1, ef21);
        cross(e2, f2, ef22);
        cross(e2, f3, ef23);
        cross(e3, f1, ef31);
        cross(e3, f2, ef32);
        cross(e3, f3, ef33);

        // now begin the series of tests

        if (!project6(n1, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(m1, p1, p2, p3, q1, q2, q3)) return false;

        if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return false;

        if (!project6(g1, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(g2, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(g3, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(h1, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(h2, p1, p2, p3, q1, q2, q3)) return false;
        if (!project6(h3, p1, p2, p3, q1, q2, q3)) return false;

        return true;
    }

}
}

#endif /* TRIDEVILLER_HPP_ */
