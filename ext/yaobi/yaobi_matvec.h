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

#ifndef YAOBI_MAT_VEC_H_
#define YAOBI_MAT_VEC_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_matvec.h
//! \brief Several macros for vectors and transformation matrices.
//!
//! \author Morten Strandberg
//! \date 6/10 2005
//!

namespace yaobi {


//! Convenient constants for accessing transformation matrix elements.
#ifdef YAOBI_USE_SSE2
  //! make sure matrices are stored column-major order, otherwise they cannot
  //! be used with SSE2 instructions.
  enum {
    R00 = 0,  R10 = 1,  R20 = 2,
    R01 = 4,  R11 = 5,  R21 = 6,
    R02 = 8,  R12 = 9,  R22 = 10,
    TX  = 12, TY  = 13, TZ  = 14,
  };  
#else
  //! \note Matrices are stored column-major order. This gives us the option
  //! to switch to SSE2 instructions, if should they prove more efficient.
  enum {
    R00 = 0, R10, R20,
    R01,     R11, R21,
    R02,     R12, R22,
    TX,      TY,  TZ
  };
#endif


//! Used in debug mode to check that rotation matrices are OK
#define ROTMATRIX_TOL ((Real) 1.0e-5)

//! Rotates a vector \a v into the vector \a vr using the rotation matrix \a r
//! \param[out] vr The rotated vector
//! \param[in]  v  A vector
//! \param[in]  r  A rotation matrix
#define ROTATE_VEC(vr,r,v)                                             \
  (vr)[0] = ((r)[R00]*(v)[0]) + ((r)[R01]*(v)[1]) + ((r)[R02]*(v)[2]); \
  (vr)[1] = ((r)[R10]*(v)[0]) + ((r)[R11]*(v)[1]) + ((r)[R12]*(v)[2]); \
  (vr)[2] = ((r)[R20]*(v)[0]) + ((r)[R21]*(v)[1]) + ((r)[R22]*(v)[2])

//! Rotates a vector \a v into the vector \a vr using the inverse of the
//! rotation matrix \a r
//! \param[out] vr The rotated vector
//! \param[in]  v  A vector
//! \param[in]  r  A rotation matrix
#define INV_ROTATE_VEC(vr,r,v)                                         \
  (vr)[0] = ((r)[R00]*(v)[0]) + ((r)[R10]*(v)[1]) + ((r)[R20]*(v)[2]); \
  (vr)[1] = ((r)[R01]*(v)[0]) + ((r)[R11]*(v)[1]) + ((r)[R21]*(v)[2]); \
  (vr)[2] = ((r)[R02]*(v)[0]) + ((r)[R12]*(v)[1]) + ((r)[R22]*(v)[2])


//! Transforms a vector \a v into the vector \a vt using the
//! transformation matrix \a t
//! \param[out] vt The transformed vector
//! \param[in]  v  A vector
//! \param[in]  t  A transformation matrix
#define TRANSFORM_VEC(vt,t,v)                                                    \
  (vt)[0] = ((t)[R00]*(v)[0]) + ((t)[R01]*(v)[1]) + ((t)[R02]*(v)[2]) + (t)[TX]; \
  (vt)[1] = ((t)[R10]*(v)[0]) + ((t)[R11]*(v)[1]) + ((t)[R12]*(v)[2]) + (t)[TY]; \
  (vt)[2] = ((t)[R20]*(v)[0]) + ((t)[R21]*(v)[1]) + ((t)[R22]*(v)[2]) + (t)[TZ]

//! Computes only the x-coordinate when applying the transformation matrix \a t to
//! the vector \a v. The resulting coordinate is put in \a vt.
//! \param[out] vt Vector containing the transformed x-coordinate
//! \param[in]  v  A vector
//! \param[in]  t  A transformation matrix
#define TRANSFORM_VEC_X(vt,t,v)                                                  \
  ((vt)[0] = ((t)[R00]*(v)[0]) + ((t)[R01]*(v)[1]) + ((t)[R02]*(v)[2]) + (t)[TX])

//! Computes only the y-coordinate when applying the transformation matrix \a t to
//! the vector \a v. The resulting coordinate is put in \a vt.
//! \param[out] vt Vector containing the transformed y-coordinate
//! \param[in]  v  A vector
//! \param[in]  t  A transformation matrix
#define TRANSFORM_VEC_Y(vt,t,v)                                                  \
  ((vt)[1] = ((t)[R10]*(v)[0]) + ((t)[R11]*(v)[1]) + ((t)[R12]*(v)[2]) + (t)[TY])

//! Computes only the z-coordinate when applying the transformation matrix \a t to
//! the vector \a v. The resulting coordinate is put in \a vt.
//! \param[out] vt Vector containing the transformed z-coordinate
//! \param[in]  v  A vector
//! \param[in]  t  A transformation matrix
#define TRANSFORM_VEC_Z(vt,t,v)                                                  \
  ((vt)[2] = ((t)[R20]*(v)[0]) + ((t)[R21]*(v)[1]) + ((t)[R22]*(v)[2]) + (t)[TZ])


//! Sets an identity rotation matrix.
#define SET_IDENTITY_ROT(r)                          \
  (r)[R00] = 1.0f; (r)[R01] = 0.0f; (r)[R02] = 0.0f; \
  (r)[R10] = 0.0f; (r)[R11] = 1.0f; (r)[R12] = 0.0f; \
  (r)[R20] = 0.0f; (r)[R21] = 0.0f; (r)[R22] = 1.0f

//! Sets an identity transformation matrix.
#define SET_IDENTITY_TRANSFORM(t)                                    \
  (t)[R00] = 1.0f; (t)[R01] = 0.0f; (t)[R02] = 0.0f; (t)[TX] = 0.0f; \
  (t)[R10] = 0.0f; (t)[R11] = 1.0f; (t)[R12] = 0.0f; (t)[TY] = 0.0f; \
  (t)[R20] = 0.0f; (t)[R21] = 0.0f; (t)[R22] = 1.0f; (t)[TZ] = 0.0f

//! \param[in] t A transformation matrix
//! \return The translation vector from the transformation matrix \a t
#define GET_TRANSLATION(t) ((t) + TX)

//! Multiplication of two rotation matrices.
//! \param[out] ab The product of \a a and \a b
//! \param[in]  a  A rotation matrix
//! \param[in]  b  A rotation matrix
#define ROT_MTRX_MULT(ab,a,b)                                                   \
  (ab)[R00] = ((a)[R00]*(b)[R00]) + ((a)[R01]*(b)[R10]) + ((a)[R02]*(b)[R20]);  \
  (ab)[R10] = ((a)[R10]*(b)[R00]) + ((a)[R11]*(b)[R10]) + ((a)[R12]*(b)[R20]);  \
  (ab)[R20] = ((a)[R20]*(b)[R00]) + ((a)[R21]*(b)[R10]) + ((a)[R22]*(b)[R20]);  \
  (ab)[R01] = ((a)[R00]*(b)[R01]) + ((a)[R01]*(b)[R11]) + ((a)[R02]*(b)[R21]);  \
  (ab)[R11] = ((a)[R10]*(b)[R01]) + ((a)[R11]*(b)[R11]) + ((a)[R12]*(b)[R21]);  \
  (ab)[R21] = ((a)[R20]*(b)[R01]) + ((a)[R21]*(b)[R11]) + ((a)[R22]*(b)[R21]);  \
  (ab)[R02] = ((ab)[R10]*(ab)[R21]) - ((ab)[R20]*(ab)[R11]);                    \
  (ab)[R12] = ((ab)[R20]*(ab)[R01]) - ((ab)[R00]*(ab)[R21]);                    \
  (ab)[R22] = ((ab)[R00]*(ab)[R11]) - ((ab)[R10]*(ab)[R01])

//! Multiplication of two transformation matrices.
//! \param[out] ab The product of \a a and \a b
//! \param[in]  a  A transformation matrix
//! \param[in]  b  A transformation matrix
#define TR_MULT(ab,a,b)                                                               \
  ROT_MTRX_MULT(ab,a,b);                                                              \
  (ab)[TX] = ((a)[R00]*(b)[TX])  + ((a)[R01]*(b)[TY]) + ((a)[R02]*(b)[TZ]) + (a)[TX]; \
  (ab)[TY] = ((a)[R10]*(b)[TX])  + ((a)[R11]*(b)[TY]) + ((a)[R12]*(b)[TZ]) + (a)[TY]; \
  (ab)[TZ] = ((a)[R20]*(b)[TX])  + ((a)[R21]*(b)[TY]) + ((a)[R22]*(b)[TZ]) + (a)[TZ]


//! Computes the product \f$ a^{-1}b \f$, where \a a and \a b are two
//! transformation matrices.
//! \param[out] ab The product of \f$ a^{-1} \f$ and \a b
//! \param[in]  a  A transformation matrix
//! \param[in]  b  A transformation matrix
#define TR_INV_MULT(ab,a,b)                                                                               \
  (ab)[R00] = (a)[R00]*(b)[R00] + (a)[R10]*(b)[R10] + (a)[R20]*(b)[R20];                                  \
  (ab)[R10] = (a)[R01]*(b)[R00] + (a)[R11]*(b)[R10] + (a)[R21]*(b)[R20];                                  \
  (ab)[R20] = (a)[R02]*(b)[R00] + (a)[R12]*(b)[R10] + (a)[R22]*(b)[R20];                                  \
  (ab)[R01] = (a)[R00]*(b)[R01] + (a)[R10]*(b)[R11] + (a)[R20]*(b)[R21];                                  \
  (ab)[R11] = (a)[R01]*(b)[R01] + (a)[R11]*(b)[R11] + (a)[R21]*(b)[R21];                                  \
  (ab)[R21] = (a)[R02]*(b)[R01] + (a)[R12]*(b)[R11] + (a)[R22]*(b)[R21];                                  \
  (ab)[R02] = ((ab)[R10]*(ab)[R21]) - ((ab)[R20]*(ab)[R11]);                                              \
  (ab)[R12] = ((ab)[R20]*(ab)[R01]) - ((ab)[R00]*(ab)[R21]);                                              \
  (ab)[R22] = ((ab)[R00]*(ab)[R11]) - ((ab)[R10]*(ab)[R01]);                                              \
  (ab)[TX]  = (a)[R00]*((b)[TX] - (a)[TX]) + (a)[R10]*((b)[TY] - (a)[TY]) + (a)[R20]*((b)[TZ] - (a)[TZ]); \
  (ab)[TY]  = (a)[R01]*((b)[TX] - (a)[TX]) + (a)[R11]*((b)[TY] - (a)[TY]) + (a)[R21]*((b)[TZ] - (a)[TZ]); \
  (ab)[TZ]  = (a)[R02]*((b)[TX] - (a)[TX]) + (a)[R12]*((b)[TY] - (a)[TY]) + (a)[R22]*((b)[TZ] - (a)[TZ])

//! Rotation matrix assignment.
//! \param[out] dest A rotation matrix
//! \param[in]  src  A rotation matrix
#define ROT_MTRX_CPY(dest,src)                                                  \
  (dest)[R00] = (src)[R00]; (dest)[R01] = (src)[R01]; (dest)[R02] = (src)[R02]; \
  (dest)[R10] = (src)[R10]; (dest)[R11] = (src)[R11]; (dest)[R12] = (src)[R12]; \
  (dest)[R20] = (src)[R20]; (dest)[R21] = (src)[R21]; (dest)[R22] = (src)[R22]

//! Transformation matrix assignment.
//! \param[out] dest A transformation matrix
//! \param[in]  src  A transformation matrix
#define TRANSFORM_CPY(dest,src)   \
  ROT_MTRX_CPY(dest, src);        \
  (dest)[TX] = (src)[TX];         \
  (dest)[TY] = (src)[TY];         \
  (dest)[TZ] = (src)[TZ]

//! Computes the inverse of a transformation matrix.
//! \param[out] dest The inverse of \a src
//! \param[in]  src  A transformation matrix
#define TRANSFORM_INV(dest,src)                                                     \
  (dest)[R00] = (src)[R00]; (dest)[R01] = (src)[R10]; (dest)[R02] = (src)[R20];     \
  (dest)[R10] = (src)[R01]; (dest)[R11] = (src)[R11]; (dest)[R12] = (src)[R21];     \
  (dest)[R20] = (src)[R02]; (dest)[R21] = (src)[R12]; (dest)[R22] = (src)[R22];     \
  (dest)[TX] = -(src)[R00]*(src)[TX] - (src)[R10]*(src)[TY] - (src)[R20]*(src)[TZ]; \
  (dest)[TY] = -(src)[R01]*(src)[TX] - (src)[R11]*(src)[TY] - (src)[R21]*(src)[TZ]; \
  (dest)[TZ] = -(src)[R02]*(src)[TX] - (src)[R12]*(src)[TY] - (src)[R22]*(src)[TZ]



//! returns the dot product of the vectors \a a and \a b
//! \param[in] a A vector
//! \param[in] b A vector
#define DOT_PROD(a,b) ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])


//! the cross product of two vectors \a u and \a v
//! \param[out] uv The cross product of \a u and \a v
//! \param[in]  u  A vector
//! \param[in]  v  A vector
#define CROSS_PROD(uv,u,v)                       \
  (uv)[0] = ((u)[1]*(v)[2]) - ((u)[2]*(v)[1]);   \
  (uv)[1] = ((u)[2]*(v)[0]) - ((u)[0]*(v)[2]);   \
  (uv)[2] = ((u)[0]*(v)[1]) - ((u)[1]*(v)[0])


//! sets the elements of the vector \a v
//! \param[out] v     A vector
//! \param[in]  x,y,z Scalars
#define VEC_SET(v,x,y,z) \
  (v)[0] = (x);          \
  (v)[1] = (y);          \
  (v)[2] = (z)


//! computes the sum of the vectors \a u and \a v, and puts the result in \a dest
//! \param[out] dest The vector \f$ u + v \f$
//! \param[in]  u    A vector
//! \param[in]  v    A vector
#define VEC_PLUS(dest,u,v)     \
  (dest)[0] = (u)[0] + (v)[0]; \
  (dest)[1] = (u)[1] + (v)[1]; \
  (dest)[2] = (u)[2] + (v)[2]


//! computes the sum of the vectors \a u and \a v, and puts the result in \a u
//! \param[in,out] u
//! \param[in]     v
#define VEC_PLUS_ASSIGN(u,v) \
  (u)[0] += (v)[0];          \
  (u)[1] += (v)[1];          \
  (u)[2] += (v)[2]


//! computes the difference of the vectors \a u and \a v, and puts the result in \a dest
//! \param[out] dest
//! \param[in]  u
//! \param[in]  v
#define VEC_SUB(dest,u,v)      \
  (dest)[0] = (u)[0] - (v)[0]; \
  (dest)[1] = (u)[1] - (v)[1]; \
  (dest)[2] = (u)[2] - (v)[2]


//! computes the difference of the vectors \a u and \a v, and puts the result in \a u
//! \param[in,out] u
//! \param[in]     v
#define VEC_SUB_ASSIGN(u,v)  \
  (u)[0] -= (v)[0];          \
  (u)[1] -= (v)[1];          \
  (u)[2] -= (v)[2]


//! assigns the vector \a src to the vector \a dest
//! \param[out] dest
//! \param[in]  src
#define VEC_CPY(dest,src) \
  (dest)[0] = (src)[0];   \
  (dest)[1] = (src)[1];   \
  (dest)[2] = (src)[2]


} // namespace yaobi

#endif // YAOBI_MAT_VEC_H_
