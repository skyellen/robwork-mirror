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

#include <vector>
#include <cfloat>


#include "yaobi_matvec.h"
#include "yaobi_obb_node.h"
#include "yaobi_mesh_interface.h"

#define SWAP(a,b) {     \
  const Real tmp = (a); \
  (a)            = (b); \
  (b)            = tmp; \
}

#define SWAP_COLUMNS(t,c1,c2)   \
  SWAP(t[R0##c1], t[R0##c2]);   \
  SWAP(t[R1##c1], t[R1##c2]);   \
  SWAP(t[R2##c1], t[R2##c2])


#define MIN_X 0
#define MIN_Y 1
#define MIN_Z 2
#define MAX_X 3
#define MAX_Y 4
#define MAX_Z 5

#define TEST_VERTEX(vrtx)               \
  if (point[0] < minx) {                \
    minx                  = point[0];   \
    extremal_verts[MIN_X] = vrtx;       \
  }                                     \
  if (point[0] > maxx) {                \
    maxx                  = point[0];   \
    extremal_verts[MAX_X] = vrtx;       \
  }                                     \
  if (point[1] < miny) {                \
    miny                  = point[1];   \
    extremal_verts[MIN_Y] = vrtx;       \
  }                                     \
  if (point[1] > maxy) {                \
    maxy                  = point[1];   \
    extremal_verts[MAX_Y] = vrtx;       \
  }                                     \
  if (point[2] < minz) {                \
    minz                  = point[2];   \
    extremal_verts[MIN_Z] = vrtx;       \
  }                                     \
  if (point[2] > maxz) {                \
    maxz                  = point[2];   \
    extremal_verts[MAX_Z] = vrtx;       \
  }                                     \

//============================================================================


namespace {

void most_dist_points_on_aabb(const yaobi::TriMeshInterface& trim,
                              const unsigned int             verts[6],
                              unsigned int&                  v1,
                              unsigned int&                  v2);

} // anonymous namespace

//============================================================================


namespace yaobi {

void
OBB_Node::FitToTriangles(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris, Real diam_vec[3])
{
  unsigned int extremal_verts[6] = {0, };
  
  std::vector<bool> used_verts(trim.NumVertices(), false);
  IndexedTriangle tri;
  
  // project points of tris to local coordinates and find the extreme
  // values
  Real minx, miny, minz;
  Real maxx, maxy, maxz;
  Vector3 point; // transformed
  Vector3 v;     // original
  
  minx = miny = minz =  FLT_MAX;
  maxx = maxy = maxz = -FLT_MAX;
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    tri = trim.GetIndexedTriangle(indx_vec[i]);
    
    
    if (!used_verts[tri.p1]) {
      used_verts[tri.p1] = true;
      trim.GetVertex(tri.p1, v);
      
      INV_ROTATE_VEC(point, t_rel_top, v);
      
      TEST_VERTEX(tri.p1);
    }
    
    if (!used_verts[tri.p2]) {
      used_verts[tri.p2] = true;
      trim.GetVertex(tri.p2, v);
      
      INV_ROTATE_VEC(point, t_rel_top, v);
      
      TEST_VERTEX(tri.p2);
    }
    
    if (!used_verts[tri.p3]) {
      used_verts[tri.p3] = true;
      trim.GetVertex(tri.p3, v);
      
      INV_ROTATE_VEC(point, t_rel_top, v);
      
      TEST_VERTEX(tri.p3);
    }
  }
  
  const Vector3 center = {0.5f * (minx + maxx),
                          0.5f * (miny + maxy),
                          0.5f * (minz + maxz)};
  
  Real* const transl = GET_TRANSLATION(t_rel_top);
  
  ROTATE_VEC(transl, t_rel_top, center);
  
  dim[0] = 0.5f * (maxx - minx);
  dim[1] = 0.5f * (maxy - miny);
  dim[2] = 0.5f * (maxz - minz);
  
  if (diam_vec != 0) {
    if (trim.NumVertices() < 2) {
      VEC_SET(diam_vec, 0.0f, 0.0f, 0.0f);
    } else {
      unsigned int v1 = 0;
      unsigned int v2 = 0;
      
      most_dist_points_on_aabb(trim, extremal_verts, v1, v2);
      
      Vector3 p1;
      Vector3 p2;
      trim.GetVertex(v1, p1);
      trim.GetVertex(v2, p2);
      
      VEC_SUB(diam_vec, p2, p1);
    }
  }
  
  return;
}

//============================================================================


void
OBB_Node::SortDimensions()
{
  // make sure the smallest dimension is in the z-direction
  if (dim[2] > dim[0]) {
    SWAP(dim[2], dim[0]);
    SWAP_COLUMNS(t_rel_top, 2, 0);
    
    t_rel_top[R02] *= -1.0f;
    t_rel_top[R12] *= -1.0f;
    t_rel_top[R22] *= -1.0f;
  }
  
  if (dim[2] > dim[1]) {
    SWAP(dim[2], dim[1]);
    SWAP_COLUMNS(t_rel_top, 2, 1);
    
    t_rel_top[R02] *= -1.0f;
    t_rel_top[R12] *= -1.0f;
    t_rel_top[R22] *= -1.0f;
  }
  
  // make sure x-dimension is largest
  if (dim[0] < dim[1]) {
    SWAP(dim[0], dim[1]);
    SWAP_COLUMNS(t_rel_top, 0, 1);
    
    t_rel_top[R01] *= -1.0f;
    t_rel_top[R11] *= -1.0f;
    t_rel_top[R21] *= -1.0f;
  }
  
  return;  
}

} // namespace yaobi

//============================================================================


namespace {

using namespace yaobi;

void
most_dist_points_on_aabb(const TriMeshInterface& trim,
                         const unsigned int      verts[6],
                         unsigned int&           v1,
                         unsigned int&           v2)
{
  unsigned int minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
  
  Vector3 v[6];
  
  {
    for (unsigned int i = 0; i < 6; ++i) {
      trim.GetVertex(verts[i], v[i]);
    }
  }
  
  for (unsigned int i = 1; i < 6; ++i) {
    if (v[i][0] < v[minx][0]) { minx = i; }
    if (v[i][0] > v[maxx][0]) { maxx = i; }
    
    if (v[i][1] < v[miny][1]) { miny = i; }
    if (v[i][1] > v[maxy][1]) { maxy = i; }
    
    if (v[i][2] < v[minz][2]) { minz = i; }
    if (v[i][2] > v[maxz][2]) { maxz = i; }
  }
  
  Vector3 diff;
  
  VEC_SUB(diff, v[maxx], v[minx]);
  const Real dist2x = DOT_PROD(diff, diff);
  
  VEC_SUB(diff, v[maxy], v[miny]);
  const Real dist2y = DOT_PROD(diff, diff);
  
  VEC_SUB(diff, v[maxz], v[minz]);
  const Real dist2z = DOT_PROD(diff, diff);
  
  v1 = minx;
  v2 = maxx;
  
  if (dist2y > dist2x && dist2y > dist2z) {
    v1 = miny;
    v2 = maxy;
  }
  
  if (dist2z > dist2x && dist2z > dist2y) {
    v1 = minz;
    v2 = maxz;
  }
  
  v1 = verts[v1];
  v2 = verts[v2];
  
  return;
}

} // anonymous namespace

