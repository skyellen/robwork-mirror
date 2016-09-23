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


//////////////////////////////////////////////////////////////////////////////
//
// Author: Morten Strandberg
// Date:   6/10 2005
//
//

#include <cassert>
#include <cstring>

#include "yaobi.h"
#include "yaobi_matvec.h"
#include "yaobi_obb_node.h"
#include "yaobi_fpu.h"
#include "yaobi_tri_overlap.h"
#include "yaobi_mesh_interface.h"

// used in the separating axis theorem to avoid numerical problems
#define OBB_EPS ((Real) 1.0e-6)


#ifdef YAOBI_USE_FLOAT
#  ifdef YAOBI_CPU_COMPARE
#    define GREATER(x, y)    AIR(x) > IR(y)
#  else
#    define GREATER(x, y)    FABS(x) > (y)
#  endif
#else
#  define GREATER(x, y)      FABS(x) > (y)
#endif

#define SWAP(a,b) {     \
  const Real tmp = (a); \
  (a)            = (b); \
  (b)            = tmp; \
}

#define TRANSFORM_TRIANGLE(t,verts) {                               \
  Real tmp_x;                                                       \
  Real tmp_y;                                                       \
  Real tmp_z;                                                       \
  tmp_x = verts[0][0];                                              \
  tmp_y = verts[0][1];                                              \
  tmp_z = verts[0][2];                                              \
  verts[0][0] = t[R00]*tmp_x + t[R01]*tmp_y + t[R02]*tmp_z + t[TX]; \
  verts[0][1] = t[R10]*tmp_x + t[R11]*tmp_y + t[R12]*tmp_z + t[TY]; \
  verts[0][2] = t[R20]*tmp_x + t[R21]*tmp_y + t[R22]*tmp_z + t[TZ]; \
  tmp_x = verts[1][0];                                              \
  tmp_y = verts[1][1];                                              \
  tmp_z = verts[1][2];                                              \
  verts[1][0] = t[R00]*tmp_x + t[R01]*tmp_y + t[R02]*tmp_z + t[TX]; \
  verts[1][1] = t[R10]*tmp_x + t[R11]*tmp_y + t[R12]*tmp_z + t[TY]; \
  verts[1][2] = t[R20]*tmp_x + t[R21]*tmp_y + t[R22]*tmp_z + t[TZ]; \
  tmp_x = verts[2][0];                                              \
  tmp_y = verts[2][1];                                              \
  tmp_z = verts[2][2];                                              \
  verts[2][0] = t[R00]*tmp_x + t[R01]*tmp_y + t[R02]*tmp_z + t[TX]; \
  verts[2][1] = t[R10]*tmp_x + t[R11]*tmp_y + t[R12]*tmp_z + t[TY]; \
  verts[2][2] = t[R20]*tmp_x + t[R21]*tmp_y + t[R22]*tmp_z + t[TZ]; \
}


#define DO_TRI_TRI_TEST(id_t1, id_t2)                   \
  ++res.num_tri_tests;                                  \
  if (tri_tri_overlap_3d(tri_verts_a, tri_verts_b)) {   \
    res.Add(id_t1, id_t2);                              \
    if (first_contact_only) { return YAOBI_OK; }        \
  }

// obb overlap test, most common
#define PAIR_OBB_OBB         0

// triangle-box overlap test
#define PAIR_TRINODE_OBB     1

// triangle-box overlap test
#define PAIR_OBB_TRINODE     2

// triangle-box overlap test
#define PAIR_TRI_OBB         3

// triangle-box overlap test
#define PAIR_OBB_TRI         4

// triangle overlap test
#define PAIR_TRI_TRINODE     5

// triangle overlap test
#define PAIR_TRINODE_TRI     6

// triangle overlap test
#define PAIR_TRINODE_TRINODE 7

#define PAIR_TRI_TRI         8


namespace {

using namespace yaobi;

// used by the NodeStack class
struct NodePair {
  unsigned int type; // Could try a smaller type here, but it would probably lead to an alignment penalty
  int node1;
  int node2;
};

//============================================================================


#ifndef NDEBUG
Bool is_rot_matrix(const Real t[]);
#endif

//============================================================================


// The following macros are used by the function OBB_DisjointLazy

// Note that this row is computed as the cross-product of the other two.
// This means that rows 1 and 2 must be computed first.
#define TR_INV_MULT_ROW0(ab,a,b)                                                                          \
  (ab)[R00] = ((ab)[R11]*(ab)[R22]) - ((ab)[R12]*(ab)[R21]);                                              \
  (ab)[R01] = ((ab)[R12]*(ab)[R20]) - ((ab)[R10]*(ab)[R22]);                                              \
  (ab)[R02] = ((ab)[R10]*(ab)[R21]) - ((ab)[R11]*(ab)[R20]);                                              \
  (ab)[TX]  = (a)[R00]*((b)[TX] - (a)[TX]) + (a)[R10]*((b)[TY] - (a)[TY]) + (a)[R20]*((b)[TZ] - (a)[TZ])

#define TR_INV_MULT_ROW1(ab,a,b)                                                                          \
  (ab)[R10] = (a)[R01]*(b)[R00] + (a)[R11]*(b)[R10] + (a)[R21]*(b)[R20];                                  \
  (ab)[R11] = (a)[R01]*(b)[R01] + (a)[R11]*(b)[R11] + (a)[R21]*(b)[R21];                                  \
  (ab)[R12] = (a)[R01]*(b)[R02] + (a)[R11]*(b)[R12] + (a)[R21]*(b)[R22];                                  \
  (ab)[TY]  = (a)[R01]*((b)[TX] - (a)[TX]) + (a)[R11]*((b)[TY] - (a)[TY]) + (a)[R21]*((b)[TZ] - (a)[TZ])

#define TR_INV_MULT_ROW2(ab,a,b)                                                                          \
  (ab)[R20] = (a)[R02]*(b)[R00] + (a)[R12]*(b)[R10] + (a)[R22]*(b)[R20];                                  \
  (ab)[R21] = (a)[R02]*(b)[R01] + (a)[R12]*(b)[R11] + (a)[R22]*(b)[R21];                                  \
  (ab)[R22] = (a)[R02]*(b)[R02] + (a)[R12]*(b)[R12] + (a)[R22]*(b)[R22];                                  \
  (ab)[TZ]  = (a)[R02]*((b)[TX] - (a)[TX]) + (a)[R12]*((b)[TY] - (a)[TY]) + (a)[R22]*((b)[TZ] - (a)[TZ])


YAOBI_INLINE Bool
OBB_DisjointLazy(const Transform a_rel_w, const Transform b_rel_w, const Real a[], const Real b[])
{
  Transform b_rel_a;
  Transform bf;        // bf = fabs(b_rel_a) + eps
  Real t, t2;
  
  // Class I tests
  TR_INV_MULT_ROW2(b_rel_a, a_rel_w, b_rel_w);
  
  bf[R20] = FABS(b_rel_a[R20]) + OBB_EPS;
  bf[R21] = FABS(b_rel_a[R21]) + OBB_EPS;
  bf[R22] = FABS(b_rel_a[R22]) + OBB_EPS;
  
  
  // A0 x A1 = A2
  t  = b_rel_a[TZ];
  t2 = a[2] + b[0] * bf[R20] + b[1] * bf[R21] + b[2] * bf[R22];
  if (GREATER(t, t2)) { return TRUE; }
  
  
  TR_INV_MULT_ROW1(b_rel_a, a_rel_w, b_rel_w);
  
  bf[R10] = FABS(b_rel_a[R10]) + OBB_EPS;
  bf[R11] = FABS(b_rel_a[R11]) + OBB_EPS;
  bf[R12] = FABS(b_rel_a[R12]) + OBB_EPS;
  
  // A2 x A0 = A1
  t  = b_rel_a[TY];
  t2 = a[1] + b[0] * bf[R10] + b[1] * bf[R11] + b[2] * bf[R12];
  if (GREATER(t, t2)) { return TRUE; }
  
  
  TR_INV_MULT_ROW0(b_rel_a, a_rel_w, b_rel_w);
  
  bf[R00] = FABS(b_rel_a[R00]) + OBB_EPS;
  bf[R01] = FABS(b_rel_a[R01]) + OBB_EPS;
  bf[R02] = FABS(b_rel_a[R02]) + OBB_EPS;
  
  // A1 x A2 = A0
  t  = b_rel_a[TX];
  t2 = a[0] + b[0] * bf[R00] + b[1] * bf[R01] + b[2] * bf[R02];
  if (GREATER(t, t2)) { return TRUE; }
  
  assert(is_rot_matrix(b_rel_a));
  
  // Class II tests
  
  
  // B0 x B1 = B2
  t  = b_rel_a[TX]*b_rel_a[R02] + b_rel_a[TY]*b_rel_a[R12] + b_rel_a[TZ]*b_rel_a[R22];
  t2 = b[2] + a[0] * bf[R02] + a[1] * bf[R12] + a[2] * bf[R22];
  if (GREATER(t, t2)) { return TRUE; }
  
  // B2 x B0 = B1
  t  = b_rel_a[TX]*b_rel_a[R01] + b_rel_a[TY]*b_rel_a[R11] + b_rel_a[TZ]*b_rel_a[R21];
  t2 = b[1] + a[0] * bf[R01] + a[1] * bf[R11] + a[2] * bf[R21];
  if (GREATER(t, t2)) { return TRUE; }
  
  // B1 x B2 = B0
  t  = b_rel_a[TX]*b_rel_a[R00] + b_rel_a[TY]*b_rel_a[R10] + b_rel_a[TZ]*b_rel_a[R20];
  t2 = b[0] + a[0] * bf[R00] + a[1] * bf[R10] + a[2] * bf[R20];
  if (GREATER(t, t2)) { return TRUE; }
  
  
  
  // Class III tests
  
  // A0 x B0
  t  = b_rel_a[TZ] * b_rel_a[R10] - b_rel_a[TY] * b_rel_a[R20];
  t2 = a[1] * bf[R20] + a[2] * bf[R10] + b[1] * bf[R02] + b[2] * bf[R01];
  if (GREATER(t, t2)) { return TRUE; }
  
#ifdef YAOBI_FULL_OBB_TEST
  // A0 x B1
  t  = b_rel_a[TZ] * b_rel_a[R11] - b_rel_a[TY] * b_rel_a[R21];
  t2 = a[1] * bf[R21] + a[2] * bf[R11] + b[0] * bf[R02] + b[2] * bf[R00];
  if (GREATER(t, t2)) { return TRUE; }
  
  // A0 x B2
  t  = b_rel_a[TZ] * b_rel_a[R12] - b_rel_a[TY] * b_rel_a[R22];
  t2 = a[1] * bf[R22] + a[2] * bf[R12] + b[0] * bf[R01] + b[1] * bf[R00];
  if (GREATER(t, t2)) { return TRUE; }
  
  
  // A1 x B0
  t  = b_rel_a[TX] * b_rel_a[R20] - b_rel_a[TZ] * b_rel_a[R00];
  t2 = a[0] * bf[R20] + a[2] * bf[R00] + b[1] * bf[R12] + b[2] * bf[R11];
  if (GREATER(t, t2)) { return TRUE; }
  
  // A1 x B1
  t  = b_rel_a[TX] * b_rel_a[R21] - b_rel_a[TZ] * b_rel_a[R01];
  t2 = a[0] * bf[R21] + a[2] * bf[R01] + b[0] * bf[R12] + b[2] * bf[R10];
  if (GREATER(t, t2)) { return TRUE; }
  
  // A1 x B2
  t  = b_rel_a[TX] * b_rel_a[R22] - b_rel_a[TZ] * b_rel_a[R02];
  t2 = a[0] * bf[R22] + a[2] * bf[R02] + b[0] * bf[R11] + b[1] * bf[R10];
  if (GREATER(t, t2)) { return TRUE; }
  
  
  // A2 x B0
  t  = b_rel_a[TY] * b_rel_a[R00] - b_rel_a[TX] * b_rel_a[R10];
  t2 = a[0] * bf[R10] + a[1] * bf[R00] + b[1] * bf[R22] + b[2] * bf[R21];
  if (GREATER(t, t2)) { return TRUE; }
  
  // A2 x B1
  t  = b_rel_a[TY] * b_rel_a[R01] - b_rel_a[TX] * b_rel_a[R11];
  t2 = a[0] * bf[R11] + a[1] * bf[R01] + b[0] * bf[R22] + b[2] * bf[R20];
  if (GREATER(t, t2)) { return TRUE; }
  
  // A2 x B2
  t  = b_rel_a[TY] * b_rel_a[R02] - b_rel_a[TX] * b_rel_a[R12];
  t2 = a[0] * bf[R12] + a[1] * bf[R02] + b[0] * bf[R21] + b[1] * bf[R20];
  if (GREATER(t, t2)) { return TRUE; }
#endif
  
  return FALSE;
}

//============================================================================


// returns true if the triangle intersects the axis aligned box.
Bool TriBoxCollides(const Vector3   tri_verts[],
                    const Transform tri_rel_box,
                    const Real      dim[]);



//============================================================================



// NOTE: Not successful with the SSE versions yet...
#ifdef YAOBI_USE_SSE2

#undef  TRANSFORM_VEC
#define TRANSFORM_VEC(vt,t,v) TransformVec_SSE(vt,t,v)


YAOBI_INLINE void
TransformVec_SSE(float vout[], const float tr[], const float vin[])
{
  __asm {
    mov      esi, vin
    mov      edi, vout
    
    // load columns of matrix into xmm4-7
    // Because we assume w = 1.0, no multiplication is needed for the last
    // column. We will therefore add intermediate results to this column, i.e, to emm7
    // NOTE: The length of each column is three floats => 12 bytes
    mov      edx, tr
    movups   xmm4, [edx]
    movups   xmm5, [edx + 12]
    movups   xmm6, [edx + 24]
    movups   xmm7, [edx + 36]
    
    // load vin into xmm0
    movups   xmm0, [esi]
    
    
    // move vin_x into xmm1, multiply it by the first column of the matrix
    // add the result to xmm7
    movaps   xmm1, xmm0
    shufps   xmm1, xmm1, 0x00
    mulps    xmm1, xmm4
    addps    xmm7, xmm1
    
    // repeat the process for y
    movaps   xmm1, xmm0
    shufps   xmm1, xmm1, 0x55
    mulps    xmm1, xmm5
    addps    xmm7, xmm1
    
    // repeat the process for z
    movaps   xmm1, xmm0
    shufps   xmm1, xmm1, 0xAA
    mulps    xmm1, xmm6
    addps    xmm7, xmm1
    
    
    // write the result to vout
    movups   [edi], xmm7
  }
}

//============================================================================


#undef  VEC_SUB
#define VEC_SUB(dest,u,v) VecSub_SSE(dest,u,v)


YAOBI_INLINE void
VecSub_SSE(float vout[], const float v1[], const float v2[])
{
  __asm {
    mov      esi, v1
    mov      edx, v2
    mov      edi, vout
    
    movups   xmm0, [esi]
    movups   xmm1, [edx]
    subps    xmm0, xmm1
    
    // store result in vout
    movlps   [edi], xmm0
    // fill xmm0 with the z-values
    shufps   xmm0, xmm0, 0xAA
    movss    [edi + 8], xmm0
  }
}

//============================================================================


#undef  TRANSFORM_TRIANGLE
#define TRANSFORM_TRIANGLE(t,verts) TransformTriangle_SSE(t,verts[0])


YAOBI_INLINE void
TransformTriangle_SSE(const float tr[], float tri_verts[])
{
  __asm {
    mov      edx, tr
    mov      esi, tri_verts
    
    // load columns of matrix into xmm4-7
    // Because we assume w = 1.0, no multiplication is needed for the last
    // column.
    movaps   xmm4, [edx]
    movaps   xmm5, [edx + 16]
    movaps   xmm6, [edx + 32]
    movaps   xmm7, [edx + 48]
    
    
    // load x-coordinates of first and second input vertices into xmm1 and xmm3
    movss    xmm1, [esi]
    movss    xmm3, [esi + 0x10]
    shufps   xmm1, xmm1, 0x00
    shufps   xmm3, xmm3, 0x00
    mulps    xmm1, xmm4
    mulps    xmm3, xmm4
    
    // process y
    movss    xmm0, [esi + 0x04]
    movss    xmm2, [esi + 0x14]
    shufps   xmm0, xmm0, 0x00
    shufps   xmm2, xmm2, 0x00
    mulps    xmm0, xmm5
    addps    xmm1, xmm7 // add last column
    mulps    xmm2, xmm5
    addps    xmm1, xmm0
    addps    xmm3, xmm2
    
    // process z
    movss    xmm0, [esi + 0x08]
    movss    xmm2, [esi + 0x18]
    shufps   xmm0, xmm0, 0x00
    shufps   xmm2, xmm2, 0x00
    mulps    xmm0, xmm6
    addps    xmm3, xmm7 // add last column
    mulps    xmm2, xmm6
    addps    xmm1, xmm0
    addps    xmm3, xmm2
    
    
    // write result
    movaps   [esi], xmm1
    movaps   [esi+0x10], xmm3
    
    // we have the last vertex left to process
    // repeat for p3 and q3
    
    // load q3 into xmm0, xmm1, xmm2
    movaps   xmm0, [esi + 0x20]
    movaps   xmm1, xmm0
    movaps   xmm2, xmm0
    
    // shuffle such that xmm0 contains x, xmm1 contains y and
    // xmm2 contains z
    shufps   xmm0, xmm0, 0x00
    shufps   xmm1, xmm1, 0x55
    shufps   xmm2, xmm2, 0xAA
    
    // multiply columns and add to result
    mulps    xmm0, xmm4
    mulps    xmm1, xmm5
    mulps    xmm2, xmm6
    
    addps    xmm0, xmm1
    addps    xmm2, xmm7
    addps    xmm2, xmm0
    
    // write result to p3
    movaps   [esi + 0x20], xmm2
  }
}

//============================================================================


#undef  TR_MULT
#define TR_MULT(ab,a,b) TransformMult_SSE(ab,a,b)


YAOBI_INLINE void
TransformMult_SSE(float t_out[], const float t_a[], const float t_b[])
{
  __asm {
    mov      edx, t_a
    mov      esi, t_b
    
    // load the three first columns of matrix t_a into xmm5-7
    // (don't need the fourth column until later)
    movaps   xmm5, [edx]
    movaps   xmm6, [edx + 0x10]
    movaps   xmm7, [edx + 0x20]
    
    
    // load t_b[0][0] and t_b[1][0]
    movss    xmm0, [esi]
    movss    xmm1, [esi + 0x04]
    shufps   xmm0, xmm0, 0x00     // t_b[0][0]
    shufps   xmm1, xmm1, 0x00     // t_b[1][0]
    mulps    xmm0, xmm5           // t_b[0][0] * t_a[i][0]
    mulps    xmm1, xmm6           // t_b[1][0] * t_a[i][1]
    
    // load t_b[2][0] and t_b[0][1]
    movss    xmm2, [esi + 0x08]
    movss    xmm3, [esi + 0x10]
    shufps   xmm2, xmm2, 0x00     // t_b[2][0]
    shufps   xmm3, xmm3, 0x00     // t_b[0][1]
    mulps    xmm2, xmm7           // t_b[2][0] * t_a[i][2]
    mulps    xmm3, xmm5           // t_b[0][1] * t_a[i][0]
    
    // start adding some columns
    addps    xmm0, xmm1
    mov      edi,  t_out
    addps    xmm0, xmm2
    
    // write the the first column of the resulting matrix
    movaps   [edi], xmm0
    
    // load t_b[1][1] and t_b[2][1]
    movss    xmm0, [esi + 0x14]
    movss    xmm1, [esi + 0x18]
    shufps   xmm0, xmm0, 0x00     // t_b[1][1]
    shufps   xmm1, xmm1, 0x00     // t_b[2][1]
    mulps    xmm0, xmm6           // t_b[1][1] * t_a[i][1]
    mulps    xmm1, xmm7           // t_b[2][1] * t_a[i][2]
    
    // start adding some columns
    addps    xmm0, xmm3
    addps    xmm0, xmm1
    
    // write the second column of the resulting matrix
    movaps   [edi + 0x10], xmm0
    
    // load t_b[0][2] and t_b[1][2]
    movss    xmm0, [esi + 0x20]
    movss    xmm1, [esi + 0x24]
    shufps   xmm0, xmm0, 0x00     // t_b[0][2]
    shufps   xmm1, xmm1, 0x00     // t_b[1][2]
    mulps    xmm0, xmm5           // t_b[0][2] * t_a[i][0]
    mulps    xmm1, xmm6           // t_b[1][2] * t_a[i][1]
    
    
    // load t_b[2][2] and t_b[0][3]
    movss    xmm2, [esi + 0x28]
    movss    xmm3, [esi + 0x30]
    shufps   xmm2, xmm2, 0x00     // t_b[2][2]
    shufps   xmm3, xmm3, 0x00     // t_b[0][3]
    mulps    xmm2, xmm7           // t_b[2][2] * t_a[i][2]
    mulps    xmm3, xmm5           // t_b[0][3] * t_a[i][0]
    
    // start adding some columns
    addps    xmm0, xmm1
    addps    xmm0, xmm2
    
    // write the third column of the resulting matrix
    movaps   [edi + 0x20], xmm0
    
    
    // load t_b[1][3] and t_b[2][3]
    movss    xmm0, [esi + 0x34]
    movss    xmm1, [esi + 0x38]
    shufps   xmm0, xmm0, 0x00     // t_b[1][3]
    shufps   xmm1, xmm1, 0x00     // t_b[2][3]
    mulps    xmm0, xmm6           // t_b[1][3] * t_a[i][1]
    mulps    xmm1, xmm7           // t_b[2][3] * t_a[i][2]
    
    // load the last column of t_a
    movaps   xmm2, [edx + 0x30]
    
    // start adding some columns
    addps    xmm0, xmm3
    addps    xmm1, xmm2
    addps    xmm0, xmm1
    
    // write the last column of the resulting matrix
    movaps   [edi + 0x30], xmm0
  }
}

#endif // YAOBI_USE_SSE2

} // anonymous namespace



namespace yaobi {


class NodeStack {
public:
  NodeStack(unsigned int n = 10);
  ~NodeStack();
  
  void Push(const NodePair& pair);
  
  const NodePair Pop();
  
  void Pop(unsigned int n);
  
  int IsEmpty() const;
  
  void Clear();
  
  unsigned int Size() const;
  
  // does not delete any memory if n is less than the
  // number of items on the stack
  void Reserve(unsigned int n);
  
private:
  NodeStack(const NodeStack& src);            // not defined
  NodeStack& operator=(const NodeStack& rhs); // not defined
  
  NodePair*    nodes;
  unsigned int num_alloced;
  unsigned int num_nodes;
};

//============================================================================


inline
NodeStack::NodeStack(unsigned int n):
nodes(0),
num_alloced(0),
num_nodes(0)
{
  Reserve(n);
}

//============================================================================


inline
NodeStack::~NodeStack()
{
  delete[] nodes;
}

//============================================================================


YAOBI_INLINE void
NodeStack::Push(const NodePair& pair)
{
  if (num_nodes >= num_alloced) {
    Reserve(2 * num_nodes + 1);
  }
  
  nodes[num_nodes++] = pair;
  
  return;
}

//============================================================================


YAOBI_INLINE const NodePair
NodeStack::Pop()
{
  assert(num_nodes > 0);
  
  return nodes[--num_nodes];
}

//============================================================================


YAOBI_INLINE void
NodeStack::Pop(unsigned int n)
{
  assert(n <= num_nodes);
  num_nodes -= n;
  
  return;
}

//============================================================================


inline int
NodeStack::IsEmpty() const
{
  return num_nodes == 0;
}

//============================================================================


inline void
NodeStack::Clear()
{
  num_nodes = 0;
  return;
}

//============================================================================


inline unsigned int
NodeStack::Size() const
{
  return num_nodes;
}

//============================================================================


void
NodeStack::Reserve(unsigned int n)
{
  if (n > num_alloced) {
    NodePair* const tmp = new NodePair[n];
    
    if (nodes != 0) {
      memcpy(tmp, nodes, sizeof(NodePair) * num_nodes);
      
      delete[] nodes;
    }
    
    nodes       = tmp;
    num_alloced = n;
  }
  
  return;
}

//============================================================================


void
CollideResult::Add(int i1, int i2)
{
  if (num_pairs >= num_pairs_alloced) {
    Reserve(2 * num_pairs + 8);
  }
  
  pairs[num_pairs].id1 = i1;
  pairs[num_pairs].id2 = i2;
  
  ++num_pairs;
  
  return;
}

//============================================================================


CollModel::CollModel():
tri_mesh(0),
obb_nodes(0),
num_obbs(0),
num_obbs_alloced(0),
tri_nodes(0),
num_tri_nodes(0),
num_tri_nodes_alloced(0),
state(0)
{
}

//============================================================================


CollModel::CollModel(const TriMeshInterface* mesh, ShareMode mode):
tri_mesh(0),
obb_nodes(0),
num_obbs(0),
num_obbs_alloced(0),
tri_nodes(0),
num_tri_nodes(0),
num_tri_nodes_alloced(0),
state(0)
{
  SetTriMesh(mesh, mode);
}


//============================================================================


CollModel::~CollModel()
{
  Clear();
  
  if (OwnsMesh()) {
    delete tri_mesh;
  }
}

//============================================================================


void
CollModel::SetTriMesh(const TriMeshInterface* mesh, ShareMode mode)
{
  if (mesh != 0) {
    Clear();
    
    if ((tri_mesh != 0) && OwnsMesh()) {
      delete tri_mesh;
    }
    
    tri_mesh = mesh;
    
    if (mode == SHARE_DATA) {
      state &= ~OWN_MESH;
    } else {
      state |= OWN_MESH;
    }
  }
  
  return;
}

//============================================================================


unsigned int
CollModel::MemUsage(FILE* f) const
{
  const unsigned int mem_tri_mesh  = (tri_mesh != 0 && OwnsMesh())? tri_mesh->MemUsage(0) : 0;
  const unsigned int mem_obb_nodes = sizeof(OBB_Node) * num_obbs;
  const unsigned int mem_tri_nodes = sizeof(TriNode)  * num_tri_nodes;
  
  const unsigned int total_mem = mem_obb_nodes    +
                                 mem_tri_nodes    +
                                 mem_tri_mesh     +
                                 sizeof(CollModel);
  
  if (f) {
    fprintf(f, "Total for model %p: %d bytes\n", this, total_mem);
    fprintf(f, "OBB_Nodes: %u alloced (%zu bytes each) \t==> %u bytes\n",
            num_obbs, sizeof(OBB_Node), mem_obb_nodes);
    fprintf(f, "TriNodes:  %u alloced (%zu bytes each) \t==> %u bytes\n",
            num_tri_nodes, sizeof(TriNode), mem_tri_nodes);
    
    if (tri_mesh != 0) {
      if (OwnsMesh()) {
        fprintf(f, "TriMesh has %d triangles and %d vertices and takes %u bytes\n",
                tri_mesh->NumTriangles(), tri_mesh->NumVertices(), mem_tri_mesh);
      } else {
        fprintf(f, "The TriMesh is shared and therefore excluded from the calculation\n");
      }
    }
  }
  
  return total_mem;
}

//============================================================================


void
CollModel::Clear()
{
  delete[] obb_nodes;
  delete[] tri_nodes;
  
  obb_nodes             = 0;
  num_obbs              = 0;
  num_obbs_alloced      = 0;
  
  tri_nodes             = 0;
  num_tri_nodes         = 0;
  num_tri_nodes_alloced = 0;
  
  //last_triangle         = 0;
  
  state &= ~IS_VALID;
  
  return;
}

//============================================================================


void
CollModel::ShrinkToFit()
{
  // shrink fit OBB_Nodes array
  if (num_obbs_alloced > num_obbs && obb_nodes != 0) {
    OBB_Node* const tmp = new OBB_Node[num_obbs];
    
    if (tmp != 0) {
      memcpy(tmp, obb_nodes, sizeof(OBB_Node) * num_obbs);
      delete[] obb_nodes;
      obb_nodes        = tmp;
      num_obbs_alloced = num_obbs;
    }
  }
  
  // shrink fit TriNodes array
  if (num_tri_nodes_alloced > num_tri_nodes && tri_nodes != 0) {
    TriNode* const tmp = new TriNode[num_tri_nodes];
    
    if (tmp != 0) {
      memcpy(tmp, tri_nodes, sizeof(TriNode) * num_tri_nodes);
      delete[] tri_nodes;
      tri_nodes             = tmp;
      num_tri_nodes_alloced = num_tri_nodes;
    }
  }
  
  return;
}

//============================================================================


CollideResult::CollideResult(unsigned int stack_size):
num_bv_tests(0),
num_tri_tests(0),
num_tri_box_tests(0),
num_pairs_alloced(0),
num_pairs(0),
pairs(0),
stack(0)
{
  stack = new NodeStack(stack_size);
}

//============================================================================


CollideResult::~CollideResult()
{
  delete   stack;
  delete[] pairs;
}

//============================================================================


void
CollideResult::Reserve(unsigned int n)
{
  if (n > num_pairs_alloced) {
    CollisionPair* const tmp = new CollisionPair[n];
    
    if (pairs != 0) {
      memcpy(tmp, pairs, sizeof(CollisionPair) * num_pairs);
      
      delete[] pairs;
    }
    
    pairs             = tmp;
    num_pairs_alloced = n;
  }
  
  return;
}

//============================================================================


#define TINV_MUL_T(dest,a,b)                                                                           \
  dest[R00] = a[0][0]*b[0][0] + a[1][0]*b[1][0] + a[2][0]*b[2][0];                                     \
  dest[R10] = a[0][1]*b[0][0] + a[1][1]*b[1][0] + a[2][1]*b[2][0];                                     \
  dest[R20] = a[0][2]*b[0][0] + a[1][2]*b[1][0] + a[2][2]*b[2][0];                                     \
  dest[R01] = a[0][0]*b[0][1] + a[1][0]*b[1][1] + a[2][0]*b[2][1];                                     \
  dest[R11] = a[0][1]*b[0][1] + a[1][1]*b[1][1] + a[2][1]*b[2][1];                                     \
  dest[R21] = a[0][2]*b[0][1] + a[1][2]*b[1][1] + a[2][2]*b[2][1];                                     \
  dest[R02] = (dest[R10]*dest[R21]) - (dest[R20]*dest[R11]);                                           \
  dest[R12] = (dest[R20]*dest[R01]) - (dest[R00]*dest[R21]);                                           \
  dest[R22] = (dest[R00]*dest[R11]) - (dest[R10]*dest[R01]);                                           \
  dest[TX]  = a[0][0]*(b[0][3] - a[0][3]) + a[1][0]*(b[1][3] - a[1][3]) + a[2][0]*(b[2][3] - a[2][3]); \
  dest[TY]  = a[0][1]*(b[0][3] - a[0][3]) + a[1][1]*(b[1][3] - a[1][3]) + a[2][1]*(b[2][3] - a[2][3]); \
  dest[TZ]  = a[0][2]*(b[0][3] - a[0][3]) + a[1][2]*(b[1][3] - a[1][3]) + a[2][2]*(b[2][3] - a[2][3])


#define PRINT_TRANSFORM(t)                                       \
  printf("%f  %f  %f  \t| %f\n", t[R00], t[R01], t[R02], t[TX]); \
  printf("%f  %f  %f  \t| %f\n", t[R10], t[R11], t[R12], t[TY]); \
  printf("%f  %f  %f  \t| %f\n", t[R20], t[R21], t[R22], t[TZ])


//============================================================================


int
Collide(CollideResult& res,
        const Real ta[][4], const CollModel& a,
        const Real tb[][4], const CollModel& b,
        QueryType qtype)
{
  Transform b_rel_a;
  Transform a_rel_b;
  Transform x_form; // general purpose transformation
  Vector3   tri_verts_a[3];
  Vector3   tri_verts_b[3];
  NodePair  pair;
  
  
  // triangle indices
  int tri_a, tri_b;
  
  const unsigned int first_contact_only = (qtype == FIRST_CONTACT_ONLY);
  
  res.Clear();
  
  if (!a.IsValid() || !b.IsValid()) {
    return YAOBI_INVALID_MODEL;
  }
  
  TINV_MUL_T(b_rel_a, ta, tb);
  TRANSFORM_INV(a_rel_b, b_rel_a);
  assert(is_rot_matrix(b_rel_a));
  assert(is_rot_matrix(a_rel_b));
  
  int pair_trinode_obb = PAIR_TRINODE_OBB;
  int pair_obb_trinode = PAIR_OBB_TRINODE;
  int pair_trinode_tri = PAIR_TRINODE_TRI;
  int pair_tri_trinode = PAIR_TRI_TRINODE;
  
  pair.type  = PAIR_OBB_OBB;
  pair.node1 = 0;
  pair.node2 = 0;
  
  // a collision model cannot have zero OBBs and zero TriNodes at the same time!
  if (a.num_tri_nodes == 0) {
    pair_trinode_obb = PAIR_TRI_OBB;
    pair_trinode_tri = PAIR_TRI_TRI;
  }
  if (b.num_tri_nodes == 0) {
    pair_obb_trinode = PAIR_OBB_TRI;
    pair_tri_trinode = PAIR_TRI_TRI;
  }
  
  if (a.num_obbs == 0) {
    if (b.num_obbs == 0) {
      pair.type = PAIR_TRINODE_TRINODE;
    } else {
      pair.type = PAIR_TRINODE_OBB;
    }
  } else if (b.num_obbs == 0) {
    pair.type = PAIR_OBB_TRINODE;
  }
  
  NodeStack& stack = *res.stack;
  stack.Clear();
  
  do {
    const int id1 = pair.node1;
    const int id2 = pair.node2;
    
    switch (pair.type) {
    case PAIR_OBB_OBB: // the most common case
      {
        const OBB_Node& a_obb = a.obb_nodes[id1];
        const OBB_Node& b_obb = b.obb_nodes[id2];
        
        TR_MULT(x_form, b_rel_a, b_obb.t_rel_top);
        assert(is_rot_matrix(x_form));
        
        ++res.num_bv_tests;
        if (!OBB_DisjointLazy(a_obb.t_rel_top, x_form, a_obb.dim, b_obb.dim)) {
          if (a_obb.GetSize() > b_obb.GetSize()) {
            // visit the children of 'a' first
            pair.node1 = a_obb.first_child;
            
            stack.Push(pair);
            ++pair.node1;
            if (pair.node1 <= 0) {
              pair.type  =  pair_trinode_obb;
              pair.node1 = -pair.node1;
              stack.Pop(1); // undo
            }
          } else {
            // visit the children of 'b' first
            pair.node2 = b_obb.first_child;
            
            stack.Push(pair);
            ++pair.node2;
            if (pair.node2 <= 0) {
              pair.type  =  pair_obb_trinode;
              pair.node2 = -pair.node2;
              stack.Pop(1); // undo
            }
          }
          continue;
        }
      }
      break;
    case PAIR_TRINODE_OBB:
      {
        const OBB_Node& b_obb = b.obb_nodes[id2];
        
        assert(id2 >= 0 && (unsigned int)id2 < b.num_obbs && id1 >= 0 && (unsigned int)id1 < a.num_tri_nodes);
        
        TR_INV_MULT(x_form, b_obb.t_rel_top, a_rel_b);
        assert(is_rot_matrix(x_form));
        
        tri_a = a.tri_nodes[id1].tri1;
        assert(tri_a >= 0 && (unsigned int)tri_a < a.tri_mesh->NumTriangles());
        
        a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
        
        ++res.num_tri_box_tests;
        if (TriBoxCollides(tri_verts_a, x_form, b_obb.dim)) {
          pair.type  = PAIR_TRI_OBB;
          pair.node1 = tri_a;
          pair.node2 = b_obb.first_child;
          
          stack.Push(pair);
          ++pair.node2;
          stack.Push(pair);
          
          if (pair.node2 <= 0) {
            pair.type  =  pair_tri_trinode;
            pair.node2 = -pair.node2;
            stack.Pop(2); // undo
            stack.Push(pair);
          }
        }
        
        tri_a = a.tri_nodes[id1].tri2;
        assert(tri_a == -1 || (unsigned int)tri_a < a.tri_mesh->NumTriangles());
        
        if (tri_a >= 0) {
          a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
          
          ++res.num_tri_box_tests;
          if (TriBoxCollides(tri_verts_a, x_form, b_obb.dim)) {
            pair.type  = PAIR_TRI_OBB;
            pair.node1 = tri_a;
            pair.node2 = b_obb.first_child;
            
            stack.Push(pair);
            ++pair.node2;
            stack.Push(pair);
            
            if (pair.node2 <= 0) {
              pair.type  =  pair_tri_trinode;
              pair.node2 = -pair.node2;
              stack.Pop(2); // undo
              stack.Push(pair);
            }
          }
        }
      }
      break;
    case PAIR_OBB_TRINODE:
      {
        const OBB_Node& a_obb = a.obb_nodes[id1];
        
        assert(id1 >= 0 && (unsigned int)id1 < a.num_obbs && id2 >= 0 && (unsigned int)id2 < b.num_tri_nodes);
        
        TR_INV_MULT(x_form, a_obb.t_rel_top, b_rel_a);
        assert(is_rot_matrix(x_form));
        
        tri_b = b.tri_nodes[id2].tri1;
        assert(tri_b >= 0 && (unsigned int)tri_b < b.tri_mesh->NumTriangles());
        
        b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
        
        ++res.num_tri_box_tests;
        if (TriBoxCollides(tri_verts_b, x_form, a_obb.dim)) {
          pair.type  = PAIR_OBB_TRI;
          pair.node1 = a_obb.first_child;
          pair.node2 = tri_b;
          
          stack.Push(pair);
          ++pair.node1;
          stack.Push(pair);
          
          if (pair.node1 <= 0) {
            pair.type  =  pair_trinode_tri;
            pair.node1 = -pair.node1;
            stack.Pop(2); // undo
            stack.Push(pair);
          }
        }
        
        tri_b = b.tri_nodes[id2].tri2;
        assert(tri_b < 0 || (unsigned int)tri_b < b.tri_mesh->NumTriangles());
        
        if (tri_b >= 0) {
          b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
          
          ++res.num_tri_box_tests;
          if (TriBoxCollides(tri_verts_b, x_form, a_obb.dim)) {
            pair.type  = PAIR_OBB_TRI;
            pair.node1 = a_obb.first_child;
            pair.node2 = tri_b;
            
            stack.Push(pair);
            ++pair.node1;
            stack.Push(pair);
            
            if (pair.node1 <= 0) {
              pair.type  =  pair_trinode_tri;
              pair.node1 = -pair.node1;
              stack.Pop(2); // undo
              stack.Push(pair);
            }
          }
        }
      }
      break;
    case PAIR_TRI_OBB:
      assert(id1 >= 0 && (unsigned int)id1 < a.tri_mesh->NumTriangles() && id2 >= 0 && (unsigned int)id2 < b.num_obbs);
      
      TR_INV_MULT(x_form, b.obb_nodes[id2].t_rel_top, a_rel_b);
      assert(is_rot_matrix(x_form));
      
      a.tri_mesh->GetTriangle(id1, tri_verts_a);
      
      ++res.num_tri_box_tests;
      if (TriBoxCollides(tri_verts_a, x_form, b.obb_nodes[id2].dim)) {
        //pair.node1 = id1;
        pair.node2 = b.obb_nodes[id2].first_child;
        
        stack.Push(pair);
        ++pair.node2;
        if (pair.node2 <= 0) {
          pair.type  =  pair_tri_trinode;
          pair.node2 = -pair.node2;
          stack.Pop(1); // undo
        }
        continue;
      }
      break;
    case PAIR_OBB_TRI:
      assert(id1 >= 0 && (unsigned int)id1 < a.num_obbs && id2 >= 0 && (unsigned int)id2 < b.tri_mesh->NumTriangles());
      
      TR_INV_MULT(x_form, a.obb_nodes[id1].t_rel_top, b_rel_a);
      assert(is_rot_matrix(x_form));
      
      b.tri_mesh->GetTriangle(id2, tri_verts_b);
      
      ++res.num_tri_box_tests;
      if (TriBoxCollides(tri_verts_b, x_form, a.obb_nodes[id1].dim)) {
        pair.node1 = a.obb_nodes[id1].first_child;
        //pair.node2 = id2;
        
        stack.Push(pair);
        ++pair.node1;
        if (pair.node1 <= 0) {
          pair.type  =  pair_trinode_tri;
          pair.node1 = -pair.node1;
          stack.Pop(1); // undo
        }
        continue;
      }
      break;
    case PAIR_TRI_TRINODE:
      assert((unsigned int)id2 < b.num_tri_nodes);
      
      // transform the triangle of object 'a' into the frame of object 'b'
      a.tri_mesh->GetTriangle(id1, tri_verts_a);
      TRANSFORM_TRIANGLE(a_rel_b, tri_verts_a);
      
      
      tri_b = b.tri_nodes[id2].tri1;
      b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
      
      DO_TRI_TRI_TEST(id1, tri_b);
      
      tri_b = b.tri_nodes[id2].tri2;
      if (tri_b >= 0) {
        b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
        DO_TRI_TRI_TEST(id1, tri_b);
      }
      break;
    case PAIR_TRINODE_TRI:
      assert((unsigned int)id1 < a.num_tri_nodes);
      
      // transform the triangle of object 'b' into the frame of object 'a'
      b.tri_mesh->GetTriangle(id2, tri_verts_b);
      TRANSFORM_TRIANGLE(b_rel_a, tri_verts_b);
      
      tri_a = a.tri_nodes[id1].tri1;
      a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
      DO_TRI_TRI_TEST(tri_a, id2);
      
      
      tri_a = a.tri_nodes[id1].tri2;
      if (tri_a >= 0) {
        a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
        DO_TRI_TRI_TEST(tri_a, id2);
      }
      break;
    case PAIR_TRINODE_TRINODE:
      assert((unsigned int)id1 < a.num_tri_nodes && (unsigned int)id2 < b.num_tri_nodes);
      tri_a = a.tri_nodes[id1].tri1;
      a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
      
      tri_b = b.tri_nodes[id2].tri1;
      b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
      
      // transform the triangle of object 'b' into the frame of object 'a'
      TRANSFORM_TRIANGLE(b_rel_a, tri_verts_b);
      DO_TRI_TRI_TEST(tri_a, tri_b); //  <a1, b1>
      
      tri_a = a.tri_nodes[id1].tri2;
      if (tri_a >= 0) {
        a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
        DO_TRI_TRI_TEST(tri_a, tri_b); // <a2, b1>
      }
      
      tri_b = b.tri_nodes[id2].tri2;
      if (tri_b >= 0) {
        b.tri_mesh->GetTriangle(tri_b, tri_verts_b);
        // transform the triangle of object 'b' into the frame of object 'a'
        TRANSFORM_TRIANGLE(b_rel_a, tri_verts_b);
        
        // we are already have the second triangle in the node of object 'a'
        if (tri_a >= 0) {
          DO_TRI_TRI_TEST(tri_a, tri_b); // <a2, b2>
        }
        
        
        tri_a = a.tri_nodes[id1].tri1;
        a.tri_mesh->GetTriangle(tri_a, tri_verts_a);
        DO_TRI_TRI_TEST(tri_a, tri_b); // <a1, b2>
      }
      break;
    case PAIR_TRI_TRI:
      a.tri_mesh->GetTriangle(id1, tri_verts_a);
      b.tri_mesh->GetTriangle(id2, tri_verts_b);
      
      // transform the triangle of object 'b' into the frame of object 'a'
      TRANSFORM_TRIANGLE(b_rel_a, tri_verts_b);
      DO_TRI_TRI_TEST(id1, id2);
      break;
    default:
      printf("Error in YAOBI::Collide: type = %d is unhandled in switch-statement!\n", pair.type);
      break;
    }
    
    if (stack.IsEmpty()) break;
    pair = stack.Pop();
  } while (1);
  
  return YAOBI_OK;
}

} //namespace yaobi

//============================================================================



namespace {

using namespace yaobi;

#ifndef NDEBUG
Bool
is_rot_matrix(const Real t[])
{
  Real errorVal = 0.0f;
  
  // check that the columns are orthonormal
  Real dotProd = t[R00]*t[R01] + t[R10]*t[R11] + t[R20]*t[R21];
  if (FABS(dotProd) > errorVal) { errorVal = dotProd; }
  
  dotProd = t[R00]*t[R02] + t[R10]*t[R12] + t[R20]*t[R22];
  if (FABS(dotProd) > errorVal) { errorVal = dotProd; }
  
  dotProd = t[R02]*t[R01] + t[R12]*t[R11] + t[R22]*t[R21];
  if (FABS(dotProd) > errorVal) { errorVal = dotProd; }
  
  
  // check that the determinant is +1
  Real det =
      t[R00]*(t[R11]*t[R22] - t[R12]*t[R21])
    - t[R10]*(t[R01]*t[R22] - t[R02]*t[R21])
    + t[R20]*(t[R01]*t[R12] - t[R02]*t[R11]);
  det = FABS(1.0f - det);
  if (det > errorVal) errorVal = det;
  
  return errorVal < ROTMATRIX_TOL;
}
#endif


//============================================================================


// Returns true if the plane defined by normal and offset d intersects with the
// axis-aligned box with dimensions maxbox.
YAOBI_INLINE Bool
PlaneBoxOverlap(const Real normal[], Real d, const Real maxbox[])
{
  Vector3 vmin = {-maxbox[0], -maxbox[1], -maxbox[2]};
  Vector3 vmax = { maxbox[0],  maxbox[1],  maxbox[2]};
  
  if (normal[0] <= 0.0f) { SWAP(vmin[0], vmax[0]); }
  if (normal[1] <= 0.0f) { SWAP(vmin[1], vmax[1]); }
  if (normal[2] <= 0.0f) { SWAP(vmin[2], vmax[2]); }
  
  return (DOT_PROD(normal, vmin) + d <= 0.0f) && (DOT_PROD(normal, vmax) + d >= 0.0f);
}

//============================================================================

// Acknowledgement:
//   The following macros are from Pierre Terdiman's
//   Opcode library, http://www.codercorner.com/Opcode.htm

// macro to find the min and max among three variables
#define FINDMINMAX(x0, x1, x2, min, max)   \
  min = max = x0 ;                         \
  if (x1 < min)      min = x1;             \
  else if (x1 > max) max = x1;             \
  if (x2 < min)      min = x2;             \
  else if (x2 > max) max = x2

//============================================================================


#define AXISTEST_X01(a, b, fa, fb)            \
  min = a * v0[1] - b * v0[2];                \
  max = a * v2[1] - b * v2[2];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[1] + fb * dim[2];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


#define AXISTEST_X2(a, b, fa, fb)             \
  min = a * v0[1] - b * v0[2];                \
  max = a * v1[1] - b * v1[2];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[1] + fb * dim[2];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


#define AXISTEST_Y02(a, b, fa, fb)            \
  min = b * v0[2] - a * v0[0];                \
  max = b * v2[2] - a * v2[0];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[0] + fb * dim[2];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


#define AXISTEST_Y1(a, b, fa, fb)             \
  min = b * v0[2] - a * v0[0];                \
  max = b * v1[2] - a * v1[0];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[0] + fb * dim[2];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


#define AXISTEST_Z12(a, b, fa, fb)            \
  min = a * v1[0] - b * v1[1];                \
  max = a * v2[0] - b * v2[1];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[0] + fb * dim[1];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


#define AXISTEST_Z0(a, b, fa, fb)             \
  min = a * v0[0] - b * v0[1];                \
  max = a * v1[0] - b * v1[1];                \
  if (min > max) { SWAP(min, max); }          \
  rad = fa * dim[0] + fb * dim[1];            \
  if (min > rad || max < -rad) return FALSE

//============================================================================


// compute triangle edges
// - edges lazy evaluated to take advantage of early exits
// - fabs precomputed (half less work, possible since extents are always >0)
// - customized macros to take advantage of the null component
// - axis vector discarded, possibly saves useless movs
#define IMPLEMENT_CLASS3_TESTS                  \
    Real rad;                                   \
                                                \
    const Real fey0 = FABS(e0[1]);              \
    const Real fez0 = FABS(e0[2]);              \
    AXISTEST_X01(e0[2], e0[1], fez0, fey0);     \
    const Real fex0 = FABS(e0[0]);              \
    AXISTEST_Y02(e0[2], e0[0], fez0, fex0);     \
    AXISTEST_Z12(e0[1], e0[0], fey0, fex0);     \
                                                \
    const Real fey1 = FABS(e1[1]);              \
    const Real fez1 = FABS(e1[2]);              \
    AXISTEST_X01(e1[2], e1[1], fez1, fey1);     \
    const Real fex1 = FABS(e1[0]);              \
    AXISTEST_Y02(e1[2], e1[0], fez1, fex1);     \
    AXISTEST_Z0(e1[1], e1[0], fey1, fex1);      \
                                                \
    Vector3 e2;                                 \
    VEC_SUB(e2, v0, v2);                        \
    const Real fey2 = FABS(e2[1]);              \
    const Real fez2 = FABS(e2[2]);              \
    AXISTEST_X2(e2[2], e2[1], fez2, fey2);      \
    const Real fex2 = FABS(e2[0]);              \
    AXISTEST_Y1(e2[2], e2[0], fez2, fex2);      \
    AXISTEST_Z12(e2[1], e2[0], fey2, fex2)


//============================================================================


// Returns true if the triangle defined by tri_verts intersects with the axis-aligned
// box with the dimensions dim.
// Note: The function transforms the coordinates of the triangle lazily, therefore
// the parameter tri_rel_box is needed.
Bool
TriBoxCollides(const Vector3   tri_verts[],
               const Transform tri_rel_box,
               const Real      dim[])
{
  // Use separating axis theorem to test overlap between triangle and box .
  // We need to test for overlap in these directions:
  // 1) the {x,y,z}-directions
  // 2) normal of the triangle
  // 3) crossproduct(edge from tri, {x,y,z}-directin) 
  //    this gives 3x3 = 9 more tests 
  
  // transformed triangle vertices (computed lazily)
  Vector3 v0;
  Vector3 v1;
  Vector3 v2;
  
  // First, test overlap in the {x,y,z}-directions
  Real min, max;
    
  
  // Test Z-direction first (we know the boxes are thinnest in that direction)
  TRANSFORM_VEC_Z(v0, tri_rel_box, tri_verts[0]);
  TRANSFORM_VEC_Z(v1, tri_rel_box, tri_verts[1]);
  TRANSFORM_VEC_Z(v2, tri_rel_box, tri_verts[2]);
  
#ifdef YAOBI_USE_FCOMI
  if ((MIN3(v0[2], v1[2], v2[2]) >  dim[2])   ||
      (MAX3(v0[2], v1[2], v2[2]) < -dim[2]))
  {
    return FALSE;
  }
  
  // Test Y-direction
  TRANSFORM_VEC_Y(v0, tri_rel_box, tri_verts[0]);
  TRANSFORM_VEC_Y(v1, tri_rel_box, tri_verts[1]);
  TRANSFORM_VEC_Y(v2, tri_rel_box, tri_verts[2]);
  
  if ((MIN3(v0[1], v1[1], v2[1]) >  dim[1])  ||
      (MAX3(v0[1], v1[1], v2[1]) < -dim[1]))
  {
    return FALSE;
  }
  
  // Test X-direction
  TRANSFORM_VEC_X(v0, tri_rel_box, tri_verts[0]);
  TRANSFORM_VEC_X(v1, tri_rel_box, tri_verts[1]);
  TRANSFORM_VEC_X(v2, tri_rel_box, tri_verts[2]);
  
  if ((MIN3(v0[0], v1[0], v2[0]) >  dim[0])   ||
      (MAX3(v0[0], v1[0], v2[0]) < -dim[0]))
  {
    return FALSE;
  }
  
#else
  FINDMINMAX(v0[2], v1[2], v2[2], min, max);
  if (min > dim[2] || max < -dim[2]) return FALSE;
  
  // Test Y-direction
  TRANSFORM_VEC_Y(v0, tri_rel_box, tri_verts[0]);
  TRANSFORM_VEC_Y(v1, tri_rel_box, tri_verts[1]);
  TRANSFORM_VEC_Y(v2, tri_rel_box, tri_verts[2]);
  
  FINDMINMAX(v0[1], v1[1], v2[1], min, max);
  if (min > dim[1] || max < -dim[1]) return FALSE;
  
  // Test X-direction
  TRANSFORM_VEC_X(v0, tri_rel_box, tri_verts[0]);
  TRANSFORM_VEC_X(v1, tri_rel_box, tri_verts[1]);
  TRANSFORM_VEC_X(v2, tri_rel_box, tri_verts[2]);
  
  FINDMINMAX(v0[0], v1[0], v2[0], min, max);
  if (min > dim[0] || max < -dim[0]) return FALSE;
#endif // YAOBI_USE_FCOMI
  
  
  Vector3 e0;
  Vector3 e1;
  VEC_SUB(e0, v1, v0);
  VEC_SUB(e1, v2, v1);
  
  
  // 3) "Class III" tests
  IMPLEMENT_CLASS3_TESTS;
  
  
  // 2) Test if the box intersects the plane of the triangle
  Vector3 nrml;
  CROSS_PROD(nrml, e0, e1);
  const Real d = -DOT_PROD(nrml, v0);
  
  return PlaneBoxOverlap(nrml, d, dim);
}

} // anonymous namespace

