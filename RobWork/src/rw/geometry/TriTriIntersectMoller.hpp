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

#ifndef RW_GEOMETRY_TRITRIINTERSECTMOLLER_HPP_
#define RW_GEOMETRY_TRITRIINTERSECTMOLLER_HPP_

#include <rw/geometry/Triangle.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw {
namespace geometry {

    /**
     * @brief tests if two triangles are intersecting using Thomas Mollers, 1997, no div method. The
     * code is strongly inspired (read converted to use RobWork types) from Opcode 1.3 Pierre Terdiman 2001.
     */
    template<class T=double>
    class TriTriIntersectMoller {
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
        int mNbPrimPrimTests;

    private:

        //! if OPC_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;) else no check is done (which is less robust, but faster)
        #define LOCAL_EPSILON 0.000001f

        //! sort so that a<=b
        #define SORT(a,b)           \
            if(a>b)                 \
            {                       \
                const float c=a;    \
                a=b;                \
                b=c;                \
            }

        //! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
        #define EDGE_EDGE_TEST(V0, U0, U1)                      \
            Bx = U0[i0] - U1[i0];                               \
            By = U0[i1] - U1[i1];                               \
            Cx = V0[i0] - U0[i0];                               \
            Cy = V0[i1] - U0[i1];                               \
            f  = Ay*Bx - Ax*By;                                 \
            d  = By*Cx - Bx*Cy;                                 \
            if((f>0.0f && d>=0.0f && d<=f) || (f<0.0f && d<=0.0f && d>=f))  \
            {                                                   \
                const float e=Ax*Cy - Ay*Cx;                    \
                if(f>0.0f)                                      \
                {                                               \
                    if(e>=0.0f && e<=f) return true;            \
                }                                               \
                else                                            \
                {                                               \
                    if(e<=0.0f && e>=f) return true;            \
                }                                               \
            }

        //! TO BE DOCUMENTED
        #define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)      \
        {                                                       \
            float Bx,By,Cx,Cy,d,f;                              \
            const float Ax = V1[i0] - V0[i0];                   \
            const float Ay = V1[i1] - V0[i1];                   \
            /* test edge U0,U1 against V0,V1 */                 \
            EDGE_EDGE_TEST(V0, U0, U1);                         \
            /* test edge U1,U2 against V0,V1 */                 \
            EDGE_EDGE_TEST(V0, U1, U2);                         \
            /* test edge U2,U1 against V0,V1 */                 \
            EDGE_EDGE_TEST(V0, U2, U0);                         \
        }

        //! TO BE DOCUMENTED
        #define POINT_IN_TRI(V0, U0, U1, U2)                    \
        {                                                       \
            /* is T1 completly inside T2? */                    \
            /* check if V0 is inside tri(U0,U1,U2) */           \
            float a  = U1[i1] - U0[i1];                         \
            float b  = -(U1[i0] - U0[i0]);                      \
            float c  = -a*U0[i0] - b*U0[i1];                    \
            float d0 = a*V0[i0] + b*V0[i1] + c;                 \
                                                                \
            a  = U2[i1] - U1[i1];                               \
            b  = -(U2[i0] - U1[i0]);                            \
            c  = -a*U1[i0] - b*U1[i1];                          \
            const float d1 = a*V0[i0] + b*V0[i1] + c;           \
                                                                \
            a  = U0[i1] - U2[i1];                               \
            b  = -(U0[i0] - U2[i0]);                            \
            c  = -a*U2[i0] - b*U2[i1];                          \
            const float d2 = a*V0[i0] + b*V0[i1] + c;           \
            if(d0*d1>0.0f)                                      \
            {                                                   \
                if(d0*d2>0.0f) return true;                     \
            }                                                   \
        }


        /**
         * @brief test if two coplanar triangles are overlapping.
         * @return true if they are overlapping
         */
        bool coplanarTriTri(const rw::math::Vector3D<T>& n,
                            const rw::math::Vector3D<T>& v0,
                            const rw::math::Vector3D<T>& v1,
                            const rw::math::Vector3D<T>& v2,
                            const rw::math::Vector3D<T>& u0,
                            const rw::math::Vector3D<T>& u1,
                            const rw::math::Vector3D<T>& u2)
        {
            float A[3];
            short i0,i1;
            /* first project onto an axis-aligned plane, that maximizes the area */
            /* of the triangles, compute indices: i0,i1. */
            A[0] = fabsf(n[0]);
            A[1] = fabsf(n[1]);
            A[2] = fabsf(n[2]);
            if(A[0]>A[1])
            {
                if(A[0]>A[2])
                {
                    i0=1;      /* A[0] is greatest */
                    i1=2;
                }
                else
                {
                    i0=0;      /* A[2] is greatest */
                    i1=1;
                }
            }
            else   /* A[0]<=A[1] */
            {
                if(A[2]>A[1])
                {
                    i0=0;      /* A[2] is greatest */
                    i1=1;
                }
                else
                {
                    i0=0;      /* A[1] is greatest */
                    i1=2;
                }
            }

            /* test all edges of triangle 1 against the edges of triangle 2 */
            EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
            EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
            EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

            /* finally, test if tri1 is totally contained in tri2 or vice versa */
            POINT_IN_TRI(v0, u0, u1, u2);
            POINT_IN_TRI(u0, v0, v1, v2);

            return false;
        }



    };

    template<class T>
    bool TriTriIntersectMoller<T>::inCollision(const rw::geometry::Triangle<value_type>& P,
                                     const rw::geometry::Triangle<value_type>& Q)
    {
        return inCollision(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
    }

    template<class T>
    bool TriTriIntersectMoller<T>::inCollision(const rw::geometry::Triangle<value_type>& P,
                                     const rw::geometry::Triangle<value_type>& Q,
                                     const rw::math::Transform3D<value_type>& PTQ)
    {
        return inCollision(P[0], P[1], P[2], PTQ*Q[0], PTQ*Q[1], PTQ*Q[2]);
    }

    template<class T>
    bool TriTriIntersectMoller<T>::inCollision(const rw::math::Vector3D<T>& P0,
                                     const rw::math::Vector3D<T>& P1,
                                     const rw::math::Vector3D<T>& P2,
                                     const rw::math::Vector3D<T>& Q0,
                                     const rw::math::Vector3D<T>& Q1,
                                     const rw::math::Vector3D<T>& Q2)
    {
        using namespace rw::math;
        // Stats
        mNbPrimPrimTests++;

        // Compute plane equation of triangle(P0,P1,P2)
        Vector3D<T> E1 = P1 - P0;
        Vector3D<T> E2 = P2 - P0;
        const Vector3D<T> N1 = cross(E1 , E2);
        const T d1 = dot(-N1,P0);
        // Plane equation 1: N1.X+d1=0

        // Put Q0,Q1,Q2 into plane equation 1 to compute signed distances to the plane
        T dQ0 = dot(N1,Q0) + d1;
        T dQ1 = dot(N1,Q1) + d1;
        T dQ2 = dot(N1,Q2) + d1;

        // Coplanarity robustness check
        if(fabsf(dQ0)<LOCAL_EPSILON) dQ0 = 0.0f;
        if(fabsf(dQ1)<LOCAL_EPSILON) dQ1 = 0.0f;
        if(fabsf(dQ2)<LOCAL_EPSILON) dQ2 = 0.0f;

        const T dQ0dQ1 = dQ0 * dQ1;
        const T dQ0dQ2 = dQ0 * dQ2;

        if(dQ0dQ1>0.0f && dQ0dQ2>0.0f)  // same sign on all of them + not equal 0 ?
            return false;               // no intersection occurs

        // Compute plane of triangle (Q0,Q1,Q2)
        E1 = Q1 - Q0;
        E2 = Q2 - Q0;
        const Vector3D<T> N2 = cross(E1 , E2);
        const T d2= dot(-N2 , Q0);
        // plane equation 2: N2.X+d2=0

        // put P0,P1,P2 into plane equation 2
        T dP0 = dot(N2,P0) + d2;
        T dP1 = dot(N2,P1) + d2;
        T dP2 = dot(N2,P2) + d2;

        // Coplanarity robustness check
        if(fabsf(dP0)<LOCAL_EPSILON) dP0 = 0.0f;
        if(fabsf(dP1)<LOCAL_EPSILON) dP1 = 0.0f;
        if(fabsf(dP2)<LOCAL_EPSILON) dP2 = 0.0f;

        const T dP0dP1 = dP0 * dP1;
        const T dP0dP2 = dP0 * dP2;

        if(dP0dP1>0.0f && dP0dP2>0.0f)  // same sign on all of them + not equal 0 ?
            return false;               // no intersection occurs

        // Compute direction of intersection line
        const Vector3D<T> D = cross(N1,N2);

        // Compute and index to the largest component of D
        T max=fabsf(D[0]);
        short index=0;
        T bb=fabsf(D[1]);
        T cc=fabsf(D[2]);
        if(bb>max) max=bb,index=1;
        if(cc>max) max=cc,index=2;

        // This is the simplified projection onto L
        const T vp0 = P0[index];
        const T vp1 = P1[index];
        const T vp2 = P2[index];

        const T up0 = Q0[index];
        const T up1 = Q1[index];
        const T up2 = Q2[index];

        // Compute interval for triangle 1
        T a,b,c,x0,x1;
        //NEWCOMPUTE_INTERVALS(vp0, vp1, vp2,dP0,dP1,dP2,dP0dP1,dP0dP2, a, b, c, x0, x1);
        //NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2,  D0D1,  D0D2, A, B, C, X0, X1)
        if (dP0dP1 > 0.0f) {
            /* here we know that D0D2<=0.0 */
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */
            a = vp2;
            b = (vp0 - vp2) * dP2;
            c = (vp1 - vp2) * dP2;
            x0 = dP2 - dP0;
            x1 = dP2 - dP1;
        } else if (dP0dP2 > 0.0f) {
            /* here we know that d0d1<=0.0 */
            a = vp1;
            b = (vp0 - vp1) * dP1;
            c = (vp2 - vp1) * dP1;
            x0 = dP1 - dP0;
            x1 = dP1 - dP2;
        } else if (dP1 * dP2 > 0.0f || dP0 != 0.0f) {
            /* here we know that d0d1<=0.0 or that D0!=0.0 */
            a = vp0;
            b = (vp1 - vp0) * dP0;
            c = (vp2 - vp0) * dP0;
            x0 = dP0 - dP1;
            x1 = dP0 - dP2;
        } else if (dP1 != 0.0f) {
            a = vp1;
            b = (vp0 - vp1) * dP1;
            c = (vp2 - vp1) * dP1;
            x0 = dP1 - dP0;
            x1 = dP1 - dP2;
        } else if (dP2 != 0.0f) {
            a = vp2;
            b = (vp0 - vp2) * dP2;
            c = (vp1 - vp2) * dP2;
            x0 = dP2 - dP0;
            x1 = dP2 - dP1;
        } else {
            /* triangles are coplanar */
            return coplanarTriTri(N1, P0, P1, P2, Q0, Q1, Q2);
        }

        // Compute interval for triangle 2
        T d,e,f,y0,y1;
        //NEWCOMPUTE_INTERVALS(up0,up1,up2,dQ0,dQ1,dQ2,dQ0dQ1,dQ0dQ2,d,e,f,y0,y1);
        //NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2,  D0D1,  D0D2, A, B, C, X0, X1)

        if (dQ0dQ1 > 0.0f) {
            /* here we know that D0D2<=0.0 */
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */
            d = up2;
            e = (up0 - up2) * dQ2;
            f = (up1 - up2) * dQ2;
            y0 = dQ2 - dQ0;
            y1 = dQ2 - dQ1;
        } else if (dQ0dQ2 > 0.0f) {
            /* here we know that d0d1<=0.0 */
            d = up1;
            e = (up0 - up1) * dQ1;
            f = (up2 - up1) * dQ1;
            y0 = dQ1 - dQ0;
            y1 = dQ1 - dQ2;
        } else if (dQ1 * dQ2 > 0.0f || dQ0 != 0.0f) {
            /* here we know that d0d1<=0.0 or that D0!=0.0 */
            d = up0;
            e = (up1 - up0) * dQ0;
            f = (up2 - up0) * dQ0;
            y0 = dQ0 - dQ1;
            y1 = dQ0 - dQ2;
        } else if (dQ1 != 0.0f) {
            d = up1;
            e = (up0 - up1) * dQ1;
            f = (up2 - up1) * dQ1;
            y0 = dQ1 - dQ0;
            y1 = dQ1 - dQ2;
        } else if (dQ2 != 0.0f) {
            d = up2;
            e = (up0 - up2) * dQ2;
            f = (up1 - up2) * dQ2;
            y0 = dQ2 - dQ0;
            y1 = dQ2 - dP1;
        } else {
            /* triangles are coplanar */
            return coplanarTriTri(N1, P0, P1, P2, Q0, Q1, Q2);
        }


        const T xx=x0*x1;
        const T yy=y0*y1;
        const T xxyy=xx*yy;

        T isect1[2], isect2[2];

        T tmp=a*xxyy;
        isect1[0]=tmp+b*x1*yy;
        isect1[1]=tmp+c*x0*yy;

        tmp=d*xxyy;
        isect2[0]=tmp+e*xx*y1;
        isect2[1]=tmp+f*xx*y0;

        SORT(isect1[0],isect1[1]);
        SORT(isect2[0],isect2[1]);

        if(isect1[1]<isect2[0] || isect2[1]<isect1[0])
            return false;

        return true;
    }

}
}

#endif /* TRIDEVILLER_HPP_ */
