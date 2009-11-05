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

#include <cmath>
#include <cstdio>

#include "yaobi_moments.h"
#include "yaobi_matvec.h"


namespace yaobi {

void
compute_moments(const double tri_verts[][3], Moment& m)
{
  double p[3], q[3], r[3];
  double u[3], v[3], nrml[3];
  
  VEC_CPY(p, tri_verts[0]);
  VEC_CPY(q, tri_verts[1]);
  VEC_CPY(r, tri_verts[2]);
  
  // compute the area of the triangle
  VEC_SUB(u, q, p);
  VEC_SUB(v, r, p);
  CROSS_PROD(nrml, u, v);
  
  m.area = 0.5 * sqrt(DOT_PROD(nrml, nrml));
  
  // compute the mean
  m.mean[0] = (p[0] + q[0] + r[0]) / 3.0;
  m.mean[1] = (p[1] + q[1] + r[1]) / 3.0;
  m.mean[2] = (p[2] + q[2] + r[2]) / 3.0;
  
  if (m.area == 0.0) {
    // second-order components in case of zero area
    m.cov[0][0] = p[0]*p[0] + q[0]*q[0] + r[0]*r[0];
    m.cov[0][1] = p[0]*p[1] + q[0]*q[1] + r[0]*r[1];
    m.cov[0][2] = p[0]*p[2] + q[0]*q[2] + r[0]*r[2];
    m.cov[1][1] = p[1]*p[1] + q[1]*q[1] + r[1]*r[1];
    m.cov[1][2] = p[1]*p[2] + q[1]*q[2] + r[1]*r[2];
    m.cov[2][2] = p[2]*p[2] + q[2]*q[2] + r[2]*r[2];
  } else {
    // get the second-order components -- note the weighting by the area
    m.cov[0][0] = m.area * ((9.0*m.mean[0] * m.mean[0]) + p[0]*p[0] + q[0]*q[0] + r[0]*r[0]) / 12.0;
    m.cov[0][1] = m.area * ((9.0*m.mean[0] * m.mean[1]) + p[0]*p[1] + q[0]*q[1] + r[0]*r[1]) / 12.0;
    m.cov[1][1] = m.area * ((9.0*m.mean[1] * m.mean[1]) + p[1]*p[1] + q[1]*q[1] + r[1]*r[1]) / 12.0;
    m.cov[0][2] = m.area * ((9.0*m.mean[0] * m.mean[2]) + p[0]*p[2] + q[0]*q[2] + r[0]*r[2]) / 12.0;
    m.cov[1][2] = m.area * ((9.0*m.mean[1] * m.mean[2]) + p[1]*p[2] + q[1]*q[2] + r[1]*r[2]) / 12.0;
    m.cov[2][2] = m.area * ((9.0*m.mean[2] * m.mean[2]) + p[2]*p[2] + q[2]*q[2] + r[2]*r[2]) / 12.0;
  }
  
  // make sure the covariance matrix is symmetric
  m.cov[2][1] = m.cov[1][2];
  m.cov[1][0] = m.cov[0][1];
  m.cov[2][0] = m.cov[0][2];
  
  return;
}

//============================================================================


void
compute_moments(const float tri_verts[][3], Moment& m)
{
  double verts_d[3][3];
  
  VEC_CPY(verts_d[0], tri_verts[0]);
  VEC_CPY(verts_d[1], tri_verts[1]);
  VEC_CPY(verts_d[2], tri_verts[2]);
  
  compute_moments(verts_d, m);
  
  return;
}

//============================================================================


#define ROTATE(a,i,j,k,l)             \
  g       = a[i][j];                  \
  h       = a[k][l];                  \
  a[i][j] = g - (s * (h + g * tau));  \
  a[k][l] = h + (s * (g - h * tau))

#define MAX_NUM_ROT 50


void
eigen_3x3(double vout[3][3], double dout[3], double a[3][3])
{
  int n = 3;
  int j, iq, ip, i;
  double tresh, theta, tau, t, sm, s, h, g, c;
  double b[3] = {a[0][0], a[1][1], a[2][2]};
  double d[3] = {a[0][0], a[1][1], a[2][2]};
  double z[3] = {0.0f, 0.0f, 0.0f};
  double v[3][3] = {{1.0f, 0.0f, 0.0f},
                    {0.0f, 1.0f, 0.0f},
                    {0.0f, 0.0f, 1.0f}};
  
  
  for (i = 0; i < MAX_NUM_ROT; i++) {
    // sum the off-diagonal components
    sm = 0.0f;
    for (ip = 0; ip < n; ip++) {
      for (iq = ip + 1; iq < n; iq++) {
        sm += fabs(a[ip][iq]);
      }
    }
    
    if (sm == 0.0f) {
      vout[0][0] = v[0][0]; vout[0][1] = v[0][1]; vout[0][2] = v[0][2];
      vout[1][0] = v[1][0]; vout[1][1] = v[1][1]; vout[1][2] = v[1][2];
      vout[2][0] = v[2][0]; vout[2][1] = v[2][1]; vout[2][2] = v[2][2];
      
      VEC_CPY(dout, d);
      return;
    }
    
    // special treshold the first three sweeps
    tresh = (i < 3)? 0.2f * sm / (n*n) : 0.0f;
    
    
    for (ip = 0; ip < n; ip++) {
      for (iq = ip + 1; iq < n; iq++) {
        g = 100.0f * fabs(a[ip][iq]);
        
        if (i > 3                          &&
            fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
        {
          a[ip][iq] = 0.0f;
        }
        else if (fabs(a[ip][iq]) > tresh)
        {
          h = d[iq] - d[ip];
          
          if (fabs(h) + g == fabs(h)) {
              t = a[ip][iq] / h; // t = 1 / (2 * theta)
          } else {
            theta = 0.5f * h / a[ip][iq];
            t     = 1.0f / (fabs(theta) + sqrt(1.0f + theta*theta));
            if (theta < 0.0f) { t = -t; }
          }
          
          c         = 1.0f / sqrt(1.0f + t * t);
          s         = t * c;
          tau       = s / (1.0f + c);
          h         = t * a[ip][iq];
          z[ip]    -= h;
          z[iq]    += h;
          d[ip]    -= h;
          d[iq]    += h;
          a[ip][iq] = 0.0f;
          
          // cyclic jacobi method
          for (j = 0; j < ip; j++)      { ROTATE(a,j,ip,j,iq); } // rotations j in [0, ip)
          for (j = ip + 1; j < iq; j++) { ROTATE(a,ip,j,j,iq); } // rotations j in (ip, iq)
          for (j = iq + 1; j < n; j++)  { ROTATE(a,ip,j,iq,j); } // rotations j in (q, n)
          for (j = 0; j < n; j++)       { ROTATE(v,j,ip,j,iq); }
        }
      }
      
      VEC_PLUS_ASSIGN(b, z);
      VEC_CPY(d, b);
      VEC_SET(z, 0.0f, 0.0f, 0.0f);
    }
  }
  
  fprintf(stderr, "eigen: too many iterations in Jacobi transform.\n");
  
  return;
}

} // namespace yaobi
