#include "TRIDeviller.hpp"

/*
 * TRIDeviller.cpp
 *
 *  Created on: 18-02-2009
 *      Author: jimali
 */

namespace {
	inline
	void
	VcrossV(float Vr[3], const float V1[3], const float V2[3])
	{
	  Vr[0] = V1[1]*V2[2] - V1[2]*V2[1];
	  Vr[1] = V1[2]*V2[0] - V1[0]*V2[2];
	  Vr[2] = V1[0]*V2[1] - V1[1]*V2[0];
	}

	inline
	float
	VdotV(const float V1[3], const float V2[3])
	{
	  return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2]);
	}

	inline
	float
	max(float a, float b, float c)
	{
	  float t = a;
	  if (b > t) t = b;
	  if (c > t) t = c;
	  return t;
	}

	inline
	float
	min(float a, float b, float c)
	{
	  float t = a;
	  if (b < t) t = b;
	  if (c < t) t = c;
	  return t;
	}

	inline
	int
	project6(float *ax,
	         float *p1, float *p2, float *p3,
	         float *q1, float *q2, float *q3)
	{
	  float P1 = VdotV(ax, p1);
	  float P2 = VdotV(ax, p2);
	  float P3 = VdotV(ax, p3);
	  float Q1 = VdotV(ax, q1);
	  float Q2 = VdotV(ax, q2);
	  float Q3 = VdotV(ax, q3);

	  float mx1 = max(P1, P2, P3);
	  float mn1 = min(P1, P2, P3);
	  float mx2 = max(Q1, Q2, Q3);
	  float mn2 = min(Q1, Q2, Q3);

	  if (mn1 > mx2) return 0;
	  if (mn2 > mx1) return 0;
	  return 1;
	}
}


// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles
bool TRIDeviller::inCollision(float *P1, float *P2, float *P3,
							  float *Q1, float *Q2, float *Q3){
	  // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
	  // Edges are (e1,e2,e3) and (f1,f2,f3).
	  // Normals are n1 and m1
	  // Outwards are (g1,g2,g3) and (h1,h2,h3).
	  //
	  // We assume that the triangle vertices are in the same coordinate system.
	  //
	  // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

  float p1[3], p2[3], p3[3];
  float q1[3], q2[3], q3[3];
  float e1[3], e2[3], e3[3];
  float f1[3], f2[3], f3[3];
  float g1[3], g2[3], g3[3];
  float h1[3], h2[3], h3[3];
  float n1[3], m1[3];

  float ef11[3], ef12[3], ef13[3];
  float ef21[3], ef22[3], ef23[3];
  float ef31[3], ef32[3], ef33[3];

  p1[0] = P1[0] - P1[0];  p1[1] = P1[1] - P1[1];  p1[2] = P1[2] - P1[2];
  p2[0] = P2[0] - P1[0];  p2[1] = P2[1] - P1[1];  p2[2] = P2[2] - P1[2];
  p3[0] = P3[0] - P1[0];  p3[1] = P3[1] - P1[1];  p3[2] = P3[2] - P1[2];

  q1[0] = Q1[0] - P1[0];  q1[1] = Q1[1] - P1[1];  q1[2] = Q1[2] - P1[2];
  q2[0] = Q2[0] - P1[0];  q2[1] = Q2[1] - P1[1];  q2[2] = Q2[2] - P1[2];
  q3[0] = Q3[0] - P1[0];  q3[1] = Q3[1] - P1[1];  q3[2] = Q3[2] - P1[2];

  e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];

  VcrossV(n1, e1, e2);
  VcrossV(m1, f1, f2);

  VcrossV(g1, e1, n1);
  VcrossV(g2, e2, n1);
  VcrossV(g3, e3, n1);
  VcrossV(h1, f1, m1);
  VcrossV(h2, f2, m1);
  VcrossV(h3, f3, m1);

  VcrossV(ef11, e1, f1);
  VcrossV(ef12, e1, f2);
  VcrossV(ef13, e1, f3);
  VcrossV(ef21, e2, f1);
  VcrossV(ef22, e2, f2);
  VcrossV(ef23, e2, f3);
  VcrossV(ef31, e3, f1);
  VcrossV(ef32, e3, f2);
  VcrossV(ef33, e3, f3);

  // now begin the series of tests

  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return 0;

  return 1;
}


// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles
bool TRIDeviller::inCollision(const WpTriangle& P, const WpTriangle& Q){
	  // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
	  // Edges are (e1,e2,e3) and (f1,f2,f3).
	  // Normals are n1 and m1
	  // Outwards are (g1,g2,g3) and (h1,h2,h3).
	  //
	  // We assume that the triangle vertices are in the same coordinate system.
	  //
	  // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

  float p1[3], p2[3], p3[3];
  float q1[3], q2[3], q3[3];
  float e1[3], e2[3], e3[3];
  float f1[3], f2[3], f3[3];
  float g1[3], g2[3], g3[3];
  float h1[3], h2[3], h3[3];
  float n1[3], m1[3];

  float ef11[3], ef12[3], ef13[3];
  float ef21[3], ef22[3], ef23[3];
  float ef31[3], ef32[3], ef33[3];

  p1[0] = P.a.t<0>() - P.a.t<0>();  p1[1] = P.a.t<1>() - P.a.t<1>();  p1[2] = P.a.t<2>() - P.a.t<2>();
  p2[0] = P.b.t<0>() - P.a.t<0>();  p2[1] = P.b.t<1>() - P.a.t<1>();  p2[2] = P.b.t<2>() - P.a.t<2>();
  p3[0] = P.c.t<0>() - P.a.t<0>();  p3[1] = P.c.t<1>() - P.a.t<1>();  p3[2] = P.c.t<2>() - P.a.t<2>();

  q1[0] = Q.a.t<0>() - P.a.t<0>();  q1[1] = Q.a.t<1>() - P.a.t<1>();  q1[2] = Q.a.t<2>() - P.a.t<2>();
  q2[0] = Q.b.t<0>() - P.a.t<0>();  q2[1] = Q.b.t<1>() - P.a.t<1>();  q2[2] = Q.b.t<2>() - P.a.t<2>();
  q3[0] = Q.c.t<0>() - P.a.t<0>();  q3[1] = Q.c.t<1>() - P.a.t<1>();  q3[2] = Q.c.t<2>() - P.a.t<2>();

  e1[0] = p2[0] - p1[0];  e1[1] = p2[1] - p1[1];  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];  e2[1] = p3[1] - p2[1];  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];  e3[1] = p1[1] - p3[1];  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];  f1[1] = q2[1] - q1[1];  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];  f2[1] = q3[1] - q2[1];  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];  f3[1] = q1[1] - q3[1];  f3[2] = q1[2] - q3[2];

  VcrossV(n1, e1, e2);
  VcrossV(m1, f1, f2);

  VcrossV(g1, e1, n1);
  VcrossV(g2, e2, n1);
  VcrossV(g3, e3, n1);
  VcrossV(h1, f1, m1);
  VcrossV(h2, f2, m1);
  VcrossV(h3, f3, m1);

  VcrossV(ef11, e1, f1);
  VcrossV(ef12, e1, f2);
  VcrossV(ef13, e1, f3);
  VcrossV(ef21, e2, f1);
  VcrossV(ef22, e2, f2);
  VcrossV(ef23, e2, f3);
  VcrossV(ef31, e3, f1);
  VcrossV(ef32, e3, f2);
  VcrossV(ef33, e3, f3);

  // now begin the series of tests

  if (!project6(n1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(m1, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(ef11, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3)) return 0;

  if (!project6(g1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(g3, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h1, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h2, p1, p2, p3, q1, q2, q3)) return 0;
  if (!project6(h3, p1, p2, p3, q1, q2, q3)) return 0;

  return 1;
}
