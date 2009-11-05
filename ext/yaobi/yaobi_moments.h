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

#ifndef YAOBI_MOMENTS_H_
#define YAOBI_MOMENTS_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_moments.h
//! \brief Computes first and second order moments for triangle sets
//!

namespace yaobi {

struct Moment {
  double area;      //!< The area
  double mean[3];   //!< The centroid
  double cov[3][3]; //!< Second order statistics
};

//! Computes first and second order statistics for the given set of triangles
void compute_moments(const double tri_verts[][3], Moment& m);

//! Computes first and second order statistics for the given set of triangles
void compute_moments(const float tri_verts[][3], Moment& m);

//! Sets all members to zero.
void clear_moments(Moment& m);

//! Accumulates the second order moments, the area, and the area-weighted mean.
void accum_moments(Moment& a, const Moment& b);

//! Computes the eigen values and the eigen vectors of the 3x3 matrix \a a.
//! The eigen vectors end up in \a vout, and the eigen values in \a dout.
void eigen_3x3(double vout[3][3], double dout[3], double a[3][3]);


//////////////////////////////////////////////////////////////////////////////
/////////////////          Inline definitions below          /////////////////


inline void
clear_moments(Moment& m)
{
  m.area = 0.0;
  
  m.mean[0] = 0.0;
  m.mean[1] = 0.0;
  m.mean[2] = 0.0;
  
  m.cov[0][0] = 0.0; m.cov[0][1] = 0.0; m.cov[0][2] = 0.0;
  m.cov[1][0] = 0.0; m.cov[1][1] = 0.0; m.cov[1][2] = 0.0;
  m.cov[2][0] = 0.0; m.cov[2][1] = 0.0; m.cov[2][2] = 0.0;
  
  return;
}

//============================================================================


inline void
accum_moments(Moment& a, const Moment& b)
{
  a.mean[0] += b.mean[0] * b.area;
  a.mean[1] += b.mean[1] * b.area;
  a.mean[2] += b.mean[2] * b.area;
  
  a.cov[0][0] += b.cov[0][0];
  a.cov[0][1] += b.cov[0][1];
  a.cov[0][2] += b.cov[0][2];
  a.cov[1][0] += b.cov[1][0];
  a.cov[1][1] += b.cov[1][1];
  a.cov[1][2] += b.cov[1][2];
  a.cov[2][0] += b.cov[2][0];
  a.cov[2][1] += b.cov[2][1];
  a.cov[2][2] += b.cov[2][2];
  
  a.area += b.area;
  
  return;
}

} //namespace yaobi


#endif // YAOBI_MOMENTS_H_
