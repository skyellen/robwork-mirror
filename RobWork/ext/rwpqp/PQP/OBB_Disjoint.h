/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_OBB_DISJOINT
#define PQP_OBB_DISJOINT

#include "MatVec.h"
#include "PQP_Compile.h"


namespace PQP {

// int
// obb_disjoint(PQP_REAL B[3][3], PQP_REAL T[3], PQP_REAL a[3], PQP_REAL b[3]);
//
// This is a test between two boxes, box A and box B.  It is assumed that
// the coordinate system is aligned and centered on box A.  The 3x3
// matrix B specifies box B's orientation with respect to box A.
// Specifically, the columns of B are the basis vectors (axis vectors) of
// box B.  The center of box B is located at the vector T.  The
// dimensions of box B are given in the array b.  The orientation and
// placement of box A, in this coordinate system, are the identity matrix
// and zero vector, respectively, so they need not be specified.  The
// dimensions of box A are given in array a.
#ifdef HW_OBB_TEST

inline
int
obb_disjoint(PQP_REAL R_AB[3][3], PQP_REAL P_AB[3], PQP_REAL a[3], PQP_REAL b[3])
{
    int i,j;
    PQP_REAL R_BA[3][3];
    PQP_REAL P_BA[3];

    // R_BA = Transpose(R_AB)
    for(i=0;i<3;i++)
        for(j=0;j<3;j++)
            R_BA[i][j] = R_AB[j][i];

    // BTA_P = -Transpose(R_AB)*P_AB ~ 9 mult
    P_BA[0] = -( R_BA[0][0]*P_AB[0] + R_BA[0][1]*P_AB[1] + R_BA[0][2]*P_AB[2] );
    P_BA[1] = -( R_BA[1][0]*P_AB[0] + R_BA[1][1]*P_AB[1] + R_BA[1][2]*P_AB[2] );
    P_BA[2] = -( R_BA[2][0]*P_AB[0] + R_BA[2][1]*P_AB[1] + R_BA[2][2]*P_AB[2] );

    // Scale the Rotational elements A and B ~ 18 mult
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            R_BA[j][i] *= P_AB[i];
            R_AB[j][i] *= P_BA[i];
        }
    }

    // Locate the 3 surfaces on A that faces B
    // Find closest corner of B to each of the 3 surfaces of A
    PQP_REAL P[3],Pxyz[3][3],Py[3],Pz[3];
    P[0]=P_AB[0];P[1]=P_AB[1];P[2]=P_AB[2];
    for(i = 0;i<3;i++){
        if(P_AB[0]>0){//Minimize
            Pxyz[i][0] = P[0]-myfabs(R_AB[0][0]);
            Pxyz[i][1] = P[1]-myfabs(R_AB[0][1]);
            Pxyz[i][2] = P[2]-myfabs(R_AB[0][2]);
        } else {//maximize
            Pxyz[i][0] = P[0]+myfabs(R_AB[0][0]);
            Pxyz[i][1] = P[1]+myfabs(R_AB[0][1]);
            Pxyz[i][2] = P[2]+myfabs(R_AB[0][2]);
        }
    }

    // If any of these corners are the closest feature, test return if they are in collision
    if( -a[1]<= Pxyz[1] && Pxyz[1]<=a[1] && -a[2]<= Px[2] && Px[2]<=a[2]) return Px[0]<a[0];
    if( -a[0]<= Pxyz[0] && Pxyz[0]<=a[0] && -a[2]<= Px[2] && Px[2]<=a[2]) return Px[1]<a[1];
    if( -a[1]<= Pxyz[1] && Pxyz[1]<=a[1] && -a[0]<= Px[0] && Px[0]<=a[0]) return Px[2]<a[2];

    // Evaluate all (at most 3) relevant vertices. Relevant vertices all lie in two planes(surfaces).




    return 0;  // should equal 0
}


#else

inline
int
obb_disjoint(PQP_REAL B[3][3], PQP_REAL T[3], PQP_REAL a[3], PQP_REAL b[3])
{
  register PQP_REAL t, s;
  register int r;
  PQP_REAL Bf[3][3];
  const PQP_REAL reps = (PQP_REAL)1e-6;

  // Bf = fabs(B)
  Bf[0][0] = myfabs(B[0][0]);  Bf[0][0] += reps;
  Bf[0][1] = myfabs(B[0][1]);  Bf[0][1] += reps;
  Bf[0][2] = myfabs(B[0][2]);  Bf[0][2] += reps;
  Bf[1][0] = myfabs(B[1][0]);  Bf[1][0] += reps;
  Bf[1][1] = myfabs(B[1][1]);  Bf[1][1] += reps;
  Bf[1][2] = myfabs(B[1][2]);  Bf[1][2] += reps;
  Bf[2][0] = myfabs(B[2][0]);  Bf[2][0] += reps;
  Bf[2][1] = myfabs(B[2][1]);  Bf[2][1] += reps;
  Bf[2][2] = myfabs(B[2][2]);  Bf[2][2] += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint
  r = 1;

  // A1 x A2 = A0
  t = myfabs(T[0]);

  r &= (t <=
      (a[0] + b[0] * Bf[0][0] + b[1] * Bf[0][1] + b[2] * Bf[0][2]));
  if (!r) return 1;

  // B1 x B2 = B0
  s = T[0]*B[0][0] + T[1]*B[1][0] + T[2]*B[2][0];
  t = myfabs(s);

  r &= ( t <=
      (b[0] + a[0] * Bf[0][0] + a[1] * Bf[1][0] + a[2] * Bf[2][0]));
  if (!r) return 2;

  // A2 x A0 = A1
  t = myfabs(T[1]);

  r &= ( t <=
      (a[1] + b[0] * Bf[1][0] + b[1] * Bf[1][1] + b[2] * Bf[1][2]));
  if (!r) return 3;

  // A0 x A1 = A2
  t = myfabs(T[2]);

  r &= ( t <=
      (a[2] + b[0] * Bf[2][0] + b[1] * Bf[2][1] + b[2] * Bf[2][2]));
  if (!r) return 4;

  // B2 x B0 = B1
  s = T[0]*B[0][1] + T[1]*B[1][1] + T[2]*B[2][1];
  t = myfabs(s);

  r &= ( t <=
      (b[1] + a[0] * Bf[0][1] + a[1] * Bf[1][1] + a[2] * Bf[2][1]));
  if (!r) return 5;

  // B0 x B1 = B2
  s = T[0]*B[0][2] + T[1]*B[1][2] + T[2]*B[2][2];
  t = myfabs(s);

  r &= ( t <=
      (b[2] + a[0] * Bf[0][2] + a[1] * Bf[1][2] + a[2] * Bf[2][2]));
  if (!r) return 6;

  // A0 x B0
  s = T[2] * B[1][0] - T[1] * B[2][0];
  t = myfabs(s);

  r &= ( t <=
    (a[1] * Bf[2][0] + a[2] * Bf[1][0] +
     b[1] * Bf[0][2] + b[2] * Bf[0][1]));
  if (!r) return 7;

  // A0 x B1
  s = T[2] * B[1][1] - T[1] * B[2][1];
  t = myfabs(s);

  r &= ( t <=
    (a[1] * Bf[2][1] + a[2] * Bf[1][1] +
     b[0] * Bf[0][2] + b[2] * Bf[0][0]));
  if (!r) return 8;

  // A0 x B2
  s = T[2] * B[1][2] - T[1] * B[2][2];
  t = myfabs(s);

  r &= ( t <=
      (a[1] * Bf[2][2] + a[2] * Bf[1][2] +
       b[0] * Bf[0][1] + b[1] * Bf[0][0]));
  if (!r) return 9;

  // A1 x B0
  s = T[0] * B[2][0] - T[2] * B[0][0];
  t = myfabs(s);

  r &= ( t <=
      (a[0] * Bf[2][0] + a[2] * Bf[0][0] +
       b[1] * Bf[1][2] + b[2] * Bf[1][1]));
  if (!r) return 10;

  // A1 x B1
  s = T[0] * B[2][1] - T[2] * B[0][1];
  t = myfabs(s);

  r &= ( t <=
      (a[0] * Bf[2][1] + a[2] * Bf[0][1] +
       b[0] * Bf[1][2] + b[2] * Bf[1][0]));
  if (!r) return 11;

  // A1 x B2
  s = T[0] * B[2][2] - T[2] * B[0][2];
  t = myfabs(s);

  r &= (t <=
      (a[0] * Bf[2][2] + a[2] * Bf[0][2] +
       b[0] * Bf[1][1] + b[1] * Bf[1][0]));
  if (!r) return 12;

  // A2 x B0
  s = T[1] * B[0][0] - T[0] * B[1][0];
  t = myfabs(s);

  r &= (t <=
      (a[0] * Bf[1][0] + a[1] * Bf[0][0] +
       b[1] * Bf[2][2] + b[2] * Bf[2][1]));
  if (!r) return 13;

  // A2 x B1
  s = T[1] * B[0][1] - T[0] * B[1][1];
  t = myfabs(s);

  r &= ( t <=
      (a[0] * Bf[1][1] + a[1] * Bf[0][1] +
       b[0] * Bf[2][2] + b[2] * Bf[2][0]));
  if (!r) return 14;

  // A2 x B2
  s = T[1] * B[0][2] - T[0] * B[1][2];
  t = myfabs(s);

  r &= ( t <=
      (a[0] * Bf[1][2] + a[1] * Bf[0][2] +
       b[0] * Bf[2][1] + b[1] * Bf[2][0]));
  if (!r) return 15;

  return 0;  // should equal 0
}
#endif

} // End namespace

#endif




