//----------------------------------------------------------------------------
//             Yaobi - Yet Another OBB-Tree Implementation
//----------------------------------------------------------------------------
//
// Copyright (c) 2006 Morten Strandberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//----------------------------------------------------------------------------

#include "yaobi_tri_overlap.h"
#include "yaobi_matvec.h"
#include "yaobi_fpu.h"


#define ALGO_GUIGUE 1

 //! The slowest algorithm, but extremely robust. Can handle degenerate
//! triangles (lines) as well.
#define ALGO_PQP    2

#define ALGO_MOLLER 3

//! defines which algorithm is used
#define ALGO ALGO_MOLLER

//! if YAOBI_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;)
//! else no check is done (which is less robust, but faster)
//! \note This epsilon is not used together with the PQP triangle overlap test.
#define LOCAL_EPSILON ((Real) 0.000001)


namespace yaobi {

#if ALGO == ALGO_GUIGUE

const char algo_name[] = "Guigue's overlap test";

/*
*               
*  Triangle-Triangle Overlap Test Routines              
*  July, 2002                                                          
*  Updated December 2003                                                
*                                                                       
*  This file contains C implementation of algorithms for                
*  performing two and three-dimensional triangle-triangle intersection test 
*  The algorithms and underlying theory are described in                    
*                                                                           
* "Fast and Robust Triangle-Triangle Overlap Test 
*  Using Orientation Predicates"  P. Guigue - O. Devillers
*                                                 
*  Journal of Graphics Tools, 8(1), 2003                                    
*                                                                           
*  Several geometric predicates are defined.  Their parameters are all      
*  points.  Each point is an array of two or three Real precision         
*  doubleing point numbers. The geometric predicates implemented in         
*  this file are:                                                           
*                                                                           
*    int tri_tri_overlap_test_3d(p1,q1,r1,p2,q2,r2)                         
*    int tri_tri_overlap_test_2d(p1,q1,r1,p2,q2,r2)                         
*                                                                           
*    int tri_tri_intersection_test_3d(p1,q1,r1,p2,q2,r2,
*                                     coplanar,source,target)               
*                                                                           
*       is a version that computes the segment of intersection when         
*       the triangles overlap (and are not coplanar)                        
*                                                                           
*    each function returns 1 if the triangles (including their              
*    boundary) intersect, otherwise 0                                       
*                                                                           
*                                                                           
*  Other information are available from the Web page                        
*  http://www.acm.org/jgt/papers/GuigueDevillers03/                         
*                                                                           
*/

#define CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2)       \
{                                              \
  VEC_SUB(v1,p2,q1);                           \
  VEC_SUB(v2,p1,q1);                           \
  CROSS_PROD(N1,v1,v2);                        \
  VEC_SUB(v1,q2,q1);                           \
  if (DOT_PROD(v1,N1) > 0.0f) return FALSE;    \
  VEC_SUB(v1,p2,p1);                           \
  VEC_SUB(v2,r1,p1);                           \
  CROSS_PROD(N1,v1,v2);                        \
  VEC_SUB(v1,r2,p1);                           \
  if (DOT_PROD(v1,N1) > 0.0f) return FALSE;    \
  else return TRUE;                            \
}

Bool coplanar_tri_tri3d(const Real p1[3], const Real q1[3], const Real r1[3],
                        const Real p2[3], const Real q2[3], const Real r2[3],
                        const Real normal[3]);


Bool tri_tri_overlap_test_2d(const Real p1[2], const Real q1[2], const Real r1[2],
                             const Real p2[2], const Real q2[2], const Real r2[2]);



/* Permutation in a canonical form of T2's vertices */

#define TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) {          \
  if (dp2 > 0.0f) {                                          \
     if (dq2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2)        \
     else if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)   \
     else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) }                 \
  else if (dp2 < 0.0f) {                                     \
    if (dq2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)         \
    else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)    \
    else CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)                    \
  } else {                                                   \
    if (dq2 < 0.0f) {                                        \
      if (dr2 >= 0.0f)  CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)     \
      else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2)                  \
    }                                                        \
    else if (dq2 > 0.0f) {                                   \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)       \
      else  CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)                 \
    }                                                        \
    else  {                                                  \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)       \
      else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2)  \
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1);  \
     }}}
  


/*
*
*  Three-dimensional Triangle-Triangle Overlap Test
*
*/
Bool
tri_tri_overlap_3d(const Vector3 t1[], const Vector3 t2[])
{
  Real v1[3], v2[3];
  Real N2[3]; 
  
  const Real* const p1 = t1[0];
  const Real* const q1 = t1[1];
  const Real* const r1 = t1[2];
  
  const Real* const p2 = t2[0];
  const Real* const q2 = t2[1];
  const Real* const r2 = t2[2];
  
  /* Compute distance signs  of p1, q1 and r1 to the plane of
     triangle(p2,q2,r2) */
  
  
  VEC_SUB(v1, q2, p2);
  VEC_SUB(v2, r2, p2);
  CROSS_PROD(N2, v1, v2);
  
  const Real d2 = DOT_PROD(N2, p2);
  Real dp1      = DOT_PROD(N2, p1) - d2;
  Real dq1      = DOT_PROD(N2, q1) - d2;
  Real dr1      = DOT_PROD(N2, r1) - d2;
  
  // Coplanarity robustness check
#ifdef YAOBI_TRITRI_EPSILON_TEST
  if (FABS(dp1) < LOCAL_EPSILON) dp1 = 0.0f;
  if (FABS(dq1) < LOCAL_EPSILON) dq1 = 0.0f;
  if (FABS(dr1) < LOCAL_EPSILON) dr1 = 0.0f;
#endif
  
  
  if ((dp1 * dq1 > 0.0f) && (dp1 * dr1 > 0.0f)) return FALSE;
  
  /* Compute distance signs  of p2, q2 and r2 to the plane of
     triangle(p1,q1,r1) */
  
  
  Real N1[3];
  VEC_SUB(v1, q1, p1);
  VEC_SUB(v2, r1, p1);
  CROSS_PROD(N1, v1, v2);
  
  const Real d1 = DOT_PROD(N1, p1);
  Real dp2      = DOT_PROD(N1, p2) - d1;
  Real dq2      = DOT_PROD(N1, q2) - d1;
  Real dr2      = DOT_PROD(N1, r2) - d1;
  
  // Coplanarity robustness check
#ifdef YAOBI_TRITRI_EPSILON_TEST
  if (FABS(dp2) < LOCAL_EPSILON) dp2 = 0.0f;
  if (FABS(dq2) < LOCAL_EPSILON) dq2 = 0.0f;
  if (FABS(dr2) < LOCAL_EPSILON) dr2 = 0.0f;
#endif
  
  
  if ((dp2 * dq2 > 0.0f) && (dp2 * dr2 > 0.0f)) return FALSE;
  
  /* Permutation in a canonical form of T1's vertices */
  
  
  if (dp1 > 0.0f) {
    if (dq1 > 0.0f) {
      TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    } else if (dr1 > 0.0f) {
      TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
    } else {
      TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
  } else if (dp1 < 0.0f) {
    if (dq1 < 0.0f) {
      TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    } else if (dr1 < 0.0f) {
      TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    } else {
      TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
    }
  } else {
    if (dq1 < 0.0f) {
      if (dr1 >= 0.0f) {
        TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      } else {
        TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
      }
    } else if (dq1 > 0.0f) {
      if (dr1 > 0.0f) {
        TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      } else {
        TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
      }
    } else {
      if (dr1 > 0.0f) {
        TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      } else if (dr1 < 0.0f) {
        TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      } else {
        return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1);
      }
    }
  }
}

//============================================================================


Bool
coplanar_tri_tri3d(const Real p1[3], const Real q1[3], const Real r1[3],
                   const Real p2[3], const Real q2[3], const Real r2[3],
                   const Real normal[3])
{
  Real P1[2],Q1[2],R1[2];
  Real P2[2],Q2[2],R2[2];
  
  Real n_x, n_y, n_z;
  
  n_x = (normal[0] < 0.0f)? -normal[0] : normal[0];
  n_y = (normal[1] < 0.0f)? -normal[1] : normal[1];
  n_z = (normal[2] < 0.0f)? -normal[2] : normal[2];
  
  
  /* Projection of the triangles in 3D onto 2D such that the area of
     the projection is maximized. */
  
  
  if (( n_x > n_z ) && ( n_x >= n_y )) {
    // Project onto plane YZ
    
    P1[0] = q1[2]; P1[1] = q1[1];
    Q1[0] = p1[2]; Q1[1] = p1[1];
    R1[0] = r1[2]; R1[1] = r1[1];
    
    P2[0] = q2[2]; P2[1] = q2[1];
    Q2[0] = p2[2]; Q2[1] = p2[1];
    R2[0] = r2[2]; R2[1] = r2[1]; 
  } else if (( n_y > n_z ) && ( n_y >= n_x )) {
    // Project onto plane XZ
    
    P1[0] = q1[0]; P1[1] = q1[2];
    Q1[0] = p1[0]; Q1[1] = p1[2];
    R1[0] = r1[0]; R1[1] = r1[2];
    
    P2[0] = q2[0]; P2[1] = q2[2];
    Q2[0] = p2[0]; Q2[1] = p2[2];
    R2[0] = r2[0]; R2[1] = r2[2];
  } else {
    // Project onto plane XY
    
    P1[0] = p1[0]; P1[1] = p1[1];
    Q1[0] = q1[0]; Q1[1] = q1[1];
    R1[0] = r1[0]; R1[1] = r1[1];
    
    P2[0] = p2[0]; P2[1] = p2[1];
    Q2[0] = q2[0]; Q2[1] = q2[1];
    R2[0] = r2[0]; R2[1] = r2[1];
  }
  
  return tri_tri_overlap_test_2d(P1, Q1, R1, P2, Q2, R2);
}





/*
*
*  Two dimensional Triangle-Triangle Overlap Test    
*
*/


/* some 2D macros */

#define ORIENT_2D(a, b, c)  ((a[0]-c[0])*(b[1]-c[1])-(a[1]-c[1])*(b[0]-c[0]))


#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2) { \
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f)                         \
    if (ORIENT_2D(R2,Q2,Q1) <= 0.0f)                       \
      if (ORIENT_2D(P1,P2,Q1) > 0.0f) {                    \
    if (ORIENT_2D(P1,Q2,Q1) <= 0.0f) return TRUE;          \
    else return FALSE;} else {                             \
    if (ORIENT_2D(P1,P2,R1) >= 0.0f)                       \
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f) return TRUE;        \
      else return FALSE;                                   \
    else return FALSE;}                                    \
    else                                                   \
      if (ORIENT_2D(P1,Q2,Q1) <= 0.0f)                     \
    if (ORIENT_2D(R2,Q2,R1) <= 0.0f)                       \
      if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) return TRUE;        \
      else return FALSE;                                   \
    else return FALSE;                                     \
      else return FALSE;                                   \
  else                                                     \
    if (ORIENT_2D(R2,P2,R1) >= 0.0f)                       \
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f)                     \
    if (ORIENT_2D(P1,P2,R1) >= 0.0f) return TRUE;          \
    else return FALSE;                                     \
      else                                                 \
    if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) {                     \
      if (ORIENT_2D(R2,R1,Q2) >= 0.0f) return TRUE;        \
      else return FALSE; }                                 \
    else return FALSE;                                     \
    else return FALSE;                                     \
}



#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2) {                 \
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f) {                                     \
    if (ORIENT_2D(P1,P2,Q1) >= 0.0f) {                                   \
        if (ORIENT_2D(P1,Q1,R2) >= 0.0f) return TRUE;                    \
        else return FALSE;} else {                                       \
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f){                                  \
    if (ORIENT_2D(R1,P1,P2) >= 0.0f) return TRUE; else return FALSE;}    \
      else return FALSE; }                                               \
  } else {                                                               \
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) {                                   \
      if (ORIENT_2D(P1,P2,R1) >= 0.0f) {                                 \
    if (ORIENT_2D(P1,R1,R2) >= 0.0f) return TRUE;                        \
    else {                                                               \
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f) return TRUE; else return FALSE;}} \
      else  return FALSE; }                                              \
    else return FALSE; }}



Bool
ccw_tri_tri_intersection_2d(const Real p1[2], const Real q1[2], const Real r1[2],
                            const Real p2[2], const Real q2[2], const Real r2[2])
{
  if (ORIENT_2D(p2,q2,p1) >= 0.0f) {
    if (ORIENT_2D(q2,r2,p1) >= 0.0f) {
      if (ORIENT_2D(r2,p2,p1) >= 0.0f) { return TRUE; }
      else { INTERSECTION_TEST_EDGE(p1,q1,r1,p2,q2,r2); }
    } else {
      if (ORIENT_2D(r2,p2,p1) >= 0.0f) { INTERSECTION_TEST_EDGE(p1,q1,r1,r2,p2,q2);   }
      else                             { INTERSECTION_TEST_VERTEX(p1,q1,r1,p2,q2,r2); }
    }
  } else {
    if (ORIENT_2D(q2,r2,p1) >= 0.0f) {
      if (ORIENT_2D(r2,p2,p1) >= 0.0f) { INTERSECTION_TEST_EDGE(p1,q1,r1,q2,r2,p2);   }
      else                             { INTERSECTION_TEST_VERTEX(p1,q1,r1,q2,r2,p2); }
    } else {
      INTERSECTION_TEST_VERTEX(p1,q1,r1,r2,p2,q2);
    }
  }
}

//============================================================================


Bool
tri_tri_overlap_test_2d(const Real p1[2], const Real q1[2], const Real r1[2],
                        const Real p2[2], const Real q2[2], const Real r2[2])
{
  if (ORIENT_2D(p1,q1,r1) < 0.0f)
    if (ORIENT_2D(p2,q2,r2) < 0.0f)
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,q2,r2);
  else
    if (ORIENT_2D(p2,q2,r2) < 0.0f)
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,q2,r2);

}

#elif ALGO == ALGO_PQP

// The algorithm used by the PQP library. It is very robust, but not so fast.

const char algo_name[] = "PQP overlap test";

inline Real
max3(Real a, Real b, Real c)
{
  const Real t = (a > b)? a : b;
  return (t > c)? t : c;
}

//============================================================================


inline Real
min3(Real a, Real b, Real c)
{
  const Real t = (a < b)? a : b;
  return (t < c)? t : c;
}

//============================================================================


Bool
project6(const Real ax[], 
         const Real p1[], const Real p2[], const Real p3[], 
         const Real q1[], const Real q2[], const Real q3[])
{
  const Real P1 = DOT_PROD(ax, p1);
  const Real P2 = DOT_PROD(ax, p2);
  const Real P3 = DOT_PROD(ax, p3);
  const Real Q1 = DOT_PROD(ax, q1);
  const Real Q2 = DOT_PROD(ax, q2);
  const Real Q3 = DOT_PROD(ax, q3);
  
  if (min3(P1, P2, P3) > max3(Q1, Q2, Q3)) return FALSE;
  
  if (min3(Q1, Q2, Q3) > max3(P1, P2, P3)) return FALSE;
  
  return TRUE;
}

//============================================================================


// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles
Bool
tri_tri_overlap_3d(const Vector3 t1[], const Vector3 t2[]);
{
  // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
  // Edges are (e1,e2,e3) and (f1,f2,f3).
  // Normals are n1 and m1
  // Outwards are (g1,g2,g3) and (h1,h2,h3).
  //  
  // We assume that the triangle vertices are in the same coordinate system.
  //
  // First thing we do is establish a new c.s. so that p1 is at (0,0,0).
  
  Real p1[3], p2[3], p3[3];
  Real q1[3], q2[3], q3[3];
  Real e1[3], e2[3], e3[3];
  Real f1[3], f2[3], f3[3];
  Real g1[3], g2[3], g3[3];
  Real h1[3], h2[3], h3[3];
  Real n1[3], m1[3];
  
  Real ef11[3], ef12[3], ef13[3];
  Real ef21[3], ef22[3], ef23[3];
  Real ef31[3], ef32[3], ef33[3];
  
  p1[0] = 0.0f;                 p1[1] = 0.0f;                 p1[2] = 0.0f;
  p2[0] = t1[1][0] - t1[0][0];  p2[1] = t1[1][1] - t1[0][1];  p2[2] = t1[1][2] - t1[0][2];
  p3[0] = t1[2][0] - t1[0][0];  p3[1] = t1[2][1] - t1[0][1];  p3[2] = t1[2][2] - t1[0][2];
  
  q1[0] = t2[0][0] - t1[0][0];  q1[1] = t2[0][1] - t1[0][1];  q1[2] = t2[0][2] - t1[0][2];
  q2[0] = t2[1][0] - t1[0][0];  q2[1] = t2[1][1] - t1[0][1];  q2[2] = t2[1][2] - t1[0][2];
  q3[0] = t2[2][0] - t1[0][0];  q3[1] = t2[2][1] - t1[0][1];  q3[2] = t2[2][2] - t1[0][2];
  
  e1[0] = p2[0];          e1[1] = p2[1];          e1[2] = p2[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];
  
  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];
  
  CROSS_PROD(n1, e1, e2);
  CROSS_PROD(m1, f1, f2);
  
  CROSS_PROD(ef11, e1, f1);
  CROSS_PROD(ef12, e1, f2);
  CROSS_PROD(ef13, e1, f3);
  CROSS_PROD(ef21, e2, f1);
  CROSS_PROD(ef22, e2, f2);
  CROSS_PROD(ef23, e2, f3);
  CROSS_PROD(ef31, e3, f1);
  CROSS_PROD(ef32, e3, f2);
  CROSS_PROD(ef33, e3, f3);
  
  // now begin the series of tests
  
  if (!project6(n1, p1, p2, p3, q1, q2, q3))   return FALSE;
  if (!project6(m1, p1, p2, p3, q1, q2, q3))   return FALSE;
  
  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return FALSE;
  
  CROSS_PROD(g1, e1, n1);
  CROSS_PROD(g2, e2, n1);
  CROSS_PROD(g3, e3, n1);
  CROSS_PROD(h1, f1, m1);
  CROSS_PROD(h2, f2, m1);
  CROSS_PROD(h3, f3, m1);
  
  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return FALSE;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return FALSE;
  
  return TRUE;
}

#elif ALGO == ALGO_MOLLER

const char algo_name[] = "Moller's overlap test";

// Algorithm by Tomas Moller, 1997.
// See article "A Fast Triangle-Triangle Intersection Test",
// Journal of Graphics Tools, 2(2), 1997
//
// The code is from Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm

//! sort so that a <= b
#define SORT(a,b)             \
  if ((a) > (b)) {            \
    const Real c = (a);       \
    (a) = (b);                \
    (b) = c;                  \
  }

//! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
#define EDGE_EDGE_TEST(V0, U0, U1)                                \
  Bx = U0[i0] - U1[i0];                                           \
  By = U0[i1] - U1[i1];                                           \
  Cx = V0[i0] - U0[i0];                                           \
  Cy = V0[i1] - U0[i1];                                           \
  f  = Ay*Bx - Ax*By;                                             \
  d  = By*Cx - Bx*Cy;                                             \
  if((f > 0.0f && d >= 0.0f && d <= f) ||                         \
     (f < 0.0f && d <= 0.0f && d >= f))                           \
  {                                                               \
      const Real e = Ax*Cy - Ay*Cx;                               \
      if (f > 0.0f)                                               \
      {                                                           \
          if (e >= 0.0f && e <= f) return TRUE;                   \
      }                                                           \
      else                                                        \
      {                                                           \
          if (e <= 0.0f && e >= f) return TRUE;                   \
      }                                                           \
  }

//! TO BE DOCUMENTED
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)    \
{                                                     \
  Real Bx,By,Cx,Cy,d,f;                               \
  const Real Ax = V1[i0] - V0[i0];                    \
  const Real Ay = V1[i1] - V0[i1];                    \
  /* test edge U0,U1 against V0,V1 */                 \
  EDGE_EDGE_TEST(V0, U0, U1);                         \
  /* test edge U1,U2 against V0,V1 */                 \
  EDGE_EDGE_TEST(V0, U1, U2);                         \
  /* test edge U2,U1 against V0,V1 */                 \
  EDGE_EDGE_TEST(V0, U2, U0);                         \
}

//! TO BE DOCUMENTED
#define POINT_IN_TRI(V0, U0, U1, U2)                  \
{                                                     \
  /* is T1 completly inside T2? */                    \
  /* check if V0 is inside tri(U0,U1,U2) */           \
  Real a  = U1[i1] - U0[i1];                          \
  Real b  = -(U1[i0] - U0[i0]);                       \
  Real c  = -a*U0[i0] - b*U0[i1];                     \
  const Real d0 = a*V0[i0] + b*V0[i1] + c;            \
                                                      \
  a  = U2[i1] - U1[i1];                               \
  b  = -(U2[i0] - U1[i0]);                            \
  c  = -a*U1[i0] - b*U1[i1];                          \
  const Real d1 = a*V0[i0] + b*V0[i1] + c;            \
                                                      \
  a  = U0[i1] - U2[i1];                               \
  b  = -(U0[i0] - U2[i0]);                            \
  c  = -a*U2[i0] - b*U2[i1];                          \
  const Real d2 = a*V0[i0] + b*V0[i1] + c;            \
  if ((d0*d1 > 0.0f) && (d0*d2 > 0.0f)) return TRUE;  \
}

//! TO BE DOCUMENTED
Bool
CoplanarTriTri(const Real n[],
               const Real v0[], const Real v1[], const Real v2[],
               const Real u0[], const Real u1[], const Real u2[])
{
  Real A[3];
  unsigned short i0, i1;
  /* first project onto an axis-aligned plane, that maximizes the area */
  /* of the triangles, compute indices: i0,i1. */
  A[0] = FABS(n[0]);
  A[1] = FABS(n[1]);
  A[2] = FABS(n[2]);
  if (A[0] > A[1])
  {
      if (A[0] > A[2])
      {
          i0 = 1;      /* A[0] is greatest */
          i1 = 2;
      }
      else
      {
          i0 = 0;      /* A[2] is greatest */
          i1 = 1;
      }
  }
  else   /* A[0]<=A[1] */
  {
      if (A[2] > A[1])
      {
          i0 = 0;      /* A[2] is greatest */
          i1 = 1;
      }
      else
      {
          i0 = 0;      /* A[1] is greatest */
          i1 = 2;
      }
  }
  
  /* test all edges of triangle 1 against the edges of triangle 2 */
  EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
  EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
  EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);
  
  /* finally, test if tri1 is totally contained in tri2 or vice versa */
  POINT_IN_TRI(v0, u0, u1, u2);
  POINT_IN_TRI(u0, v0, v1, v2);
  
  return FALSE;
}

//! TO BE DOCUMENTED
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)  \
{                                                                                     \
  if (D0D1 > 0.0f)                                                                    \
  {                                                                                   \
      /* here we know that D0D2<=0.0 */                                               \
      /* that is D0, D1 are on the same side, D2 on the other or on the plane */      \
      A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;              \
  }                                                                                   \
  else if (D0D2 > 0.0f)                                                               \
  {                                                                                   \
      /* here we know that d0d1<=0.0 */                                               \
      A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;              \
  }                                                                                   \
  else if (D1*D2 > 0.0f || D0 != 0.0f)                                                \
  {                                                                                   \
      /* here we know that d0d1<=0.0 or that D0!=0.0 */                               \
      A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;              \
  }                                                                                   \
  else if (D1 != 0.0f)                                                                \
  {                                                                                   \
      A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;              \
  }                                                                                   \
  else if (D2 != 0.0f)                                                                \
  {                                                                                   \
      A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;              \
  }                                                                                   \
  else                                                                                \
  {                                                                                   \
      /* triangles are coplanar */                                                    \
      return CoplanarTriTri(N1, t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]);            \
  }                                                                                   \
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *  Triangle/triangle intersection test routine,
 *  by Tomas Moller, 1997.
 *  See article "A Fast Triangle-Triangle Intersection Test",
 *  Journal of Graphics Tools, 2(2), 1997
 *
 *  Updated June 1999: removed the divisions -- a little faster now!
 *  Updated October 1999: added {} to CROSS and SUB macros 
 *
 *  int NoDivTriTriIsect(Real V0[3],Real V1[3],Real V2[3],
 *                      Real U0[3],Real U1[3],Real U2[3])
 *
 *  \param      V0      [in] triangle 0, vertex 0
 *  \param      V1      [in] triangle 0, vertex 1
 *  \param      V2      [in] triangle 0, vertex 2
 *  \param      U0      [in] triangle 1, vertex 0
 *  \param      U1      [in] triangle 1, vertex 1
 *  \param      U2      [in] triangle 1, vertex 2
 *  \return     true if triangles overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Bool
tri_tri_overlap_3d(const Vector3 t1[], const Vector3 t2[])
{
  Real E1[3];
  Real E2[3];
  Real N1[3];
  
  // Compute plane equation of triangle(V0,V1,V2)
  VEC_SUB(E1, t1[1], t1[0]);
  VEC_SUB(E2, t1[2], t1[0]);
  CROSS_PROD(N1, E1, E2);
  
  const Real d1 = -DOT_PROD(N1, t1[0]);
  // Plane equation 1: N1.X+d1=0
  
  // Put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
  Real du0 = DOT_PROD(N1, t2[0]) + d1;
  Real du1 = DOT_PROD(N1, t2[1]) + d1;
  Real du2 = DOT_PROD(N1, t2[2]) + d1;
  
  // Coplanarity robustness check
#ifdef YAOBI_TRITRI_EPSILON_TEST
  if (FABS(du0) < LOCAL_EPSILON) du0 = 0.0f;
  if (FABS(du1) < LOCAL_EPSILON) du1 = 0.0f;
  if (FABS(du2) < LOCAL_EPSILON) du2 = 0.0f;
#endif
  const Real du0du1 = du0 * du1;
  const Real du0du2 = du0 * du2;
  
  if (du0du1 > 0.0f && du0du2 > 0.0f)  // same sign on all of them + not equal 0 ?
    return FALSE;                      // no intersection occurs
  
  // Compute plane of triangle (U0,U1,U2)
  Real N2[3];
  VEC_SUB(E1, t2[1], t2[0]);
  VEC_SUB(E2, t2[2], t2[0]);
  CROSS_PROD(N2, E1, E2);
  const Real d2 = -DOT_PROD(N2, t2[0]);
  // plane equation 2: N2.X+d2=0
  
  // put V0,V1,V2 into plane equation 2
  Real dv0 = DOT_PROD(N2, t1[0]) + d2;
  Real dv1 = DOT_PROD(N2, t1[1]) + d2;
  Real dv2 = DOT_PROD(N2, t1[2]) + d2;
  
#ifdef YAOBI_TRITRI_EPSILON_TEST
  if (FABS(dv0) < LOCAL_EPSILON) dv0 = 0.0f;
  if (FABS(dv1) < LOCAL_EPSILON) dv1 = 0.0f;
  if (FABS(dv2) < LOCAL_EPSILON) dv2 = 0.0f;
#endif
  
  const Real dv0dv1 = dv0 * dv1;
  const Real dv0dv2 = dv0 * dv2;
  
  if (dv0dv1 > 0.0f && dv0dv2 > 0.0f)  // same sign on all of them + not equal 0 ?
    return FALSE;                     // no intersection occurs
  
  // Compute direction of intersection line
  Real D[3];
  CROSS_PROD(D, N1, N2);
  
  // Compute and index to the largest component of D
  Real max           = FABS(D[0]);
  unsigned short index = 0;
  Real bb            = FABS(D[1]);
  Real cc            = FABS(D[2]);
  if (bb > max) { max = bb; index = 1; }
  if (cc > max) { max = cc; index = 2; }
  
  // This is the simplified projection onto L
  const Real vp0 = t1[0][index];
  const Real vp1 = t1[1][index];
  const Real vp2 = t1[2][index];
  
  const Real up0 = t2[0][index];
  const Real up1 = t2[1][index];
  const Real up2 = t2[2][index];
  
  // Compute interval for triangle 1
  Real a, b, c, x0, x1;
  NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);
  
  // Compute interval for triangle 2
  Real d, e, f, y0, y1;
  NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);
  
  const Real xx   = x0*x1;
  const Real yy   = y0*y1;
  const Real xxyy = xx*yy;
  
  Real isect1[2], isect2[2];
  
  Real tmp = a*xxyy;
  isect1[0]  =  tmp + b*x1*yy;
  isect1[1]  =  tmp + c*x0*yy;
  
  tmp       = d*xxyy;
  isect2[0] = tmp + e*xx*y1;
  isect2[1] = tmp + f*xx*y0;
  
  SORT(isect1[0],isect1[1]);
  SORT(isect2[0],isect2[1]);
  
  if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) return FALSE;
  return TRUE;
}

#else
#  error Unknown choice of algorithm for triangle-triangle intersection test
#endif

//============================================================================


const char*
tri_overlap_algorithm()
{
  return algo_name;
}

} // namespace yaobi
