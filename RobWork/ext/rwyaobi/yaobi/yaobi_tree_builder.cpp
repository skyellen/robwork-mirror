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

#include <stack>
#include <cmath>
#include <vector>
#include <cassert>
#include <algorithm> // sort

#include "yaobi.h"
#include "yaobi_matvec.h"
#include "yaobi_obb_node.h"
#include "yaobi_fpu.h"
#include "yaobi_moments.h"
#include "yaobi_mesh_interface.h"
#include "yaobi_tree_builder.h"


#define MIN(a,b) (((a) > (b))? (b) : (a))

namespace {

using namespace yaobi;

typedef std::vector<int> TriIndxVec;

struct BuildNode {
  TriIndxVec   tris;           // the OBB contains these triangles
  double       split_axis[3];  // the OBB will be split across this axis
  double       split_coord;    // the coordinate on the split axis
  unsigned int obb_indx;       // the index of the obb containing the triangles
};


typedef std::stack<BuildNode> BuildStack;

//--------------------- function prototypes ----------------------------------


// Given a set of triangles, an axis and a coordinate on that axis, this function
// partitions the triangles into two sets, left_child and right_child. The axial projection
// of a triangle's centroid on the given axis determines to which set it will belong. If
// one set becomes empty and the other set contains more than one triangle, then
// the triangles will be partioned arbitrarily. The set of triangles, the split axis and the
// split coordinate are all pointed out by \a parent.
void partition_triangles(const BuildNode&        parent,
                         const TriMeshInterface& trim,
                         TriIndxVec&             left_child,
                         TriIndxVec&             right_child);

// Computes the area moments for the given set of triangles
void compute_moments(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris, Moment& m_tot);


// returns also the area moments for all the triangles within the OBB
void compute_obb(OBB_Node&               obb,
                 const TriMeshInterface& trim,
                 const int               indx_vec[],
                 unsigned int            num_tris,
                 Moment&                 tris_mom);

void compute_obb(OBB_Node&               obb,
                 BuildNode&              bnode,
                 const TriMeshInterface& trim,
                 unsigned int            flags);

double tri_area_sqrd(const TriMeshInterface& trim, unsigned int tri_id);

// Uses the algorithm of Sariel Har-Peled
// Too sensitive (buggy?) at the moment.
//const OBB_Node optimal_obb(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris);

// returns TRUE if the given set of triangles is planar (within some tolerance)
Bool is_planar(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris);

bool optimal_obb_from_tris(OBB_Node& obb, const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris);

// projects the triangle centroids onto the given axis and returns the median of the projections
double compute_median(const double axis[3], const TriMeshInterface& trim, const TriIndxVec& tris);

} // anonymous namespace

//============================================================================


namespace yaobi {

int
build_obb_tree(CollModel& m, unsigned int flags)
{
  OBB_Node left_obb;
  OBB_Node right_obb;
  
  BuildStack build_stack;
  
  BuildNode parent;
  BuildNode left_child;
  BuildNode right_child;
  
  const bool use_tri_nodes      = flags & YAOBI_USE_TRI_NODES;
  const bool swap_tris          = (flags & YAOBI_LARGEST_TRI_FIRST) || (flags & YAOBI_SMALLEST_TRI_FIRST);
  const bool swap_obbs          = (flags & YAOBI_LARGEST_OBB_FIRST) || (flags & YAOBI_SMALLEST_OBB_FIRST);
  const bool smallest_obb_first = (flags & YAOBI_SMALLEST_OBB_FIRST) != 0;
  const bool smallest_tri_first = (flags & YAOBI_SMALLEST_TRI_FIRST) != 0;
  
  
  // check that 'm' has a valid mesh
  if (m.tri_mesh == 0) {
    return YAOBI_NULL_MESH;
  }
  
  const unsigned int num_tris = m.tri_mesh->NumTriangles();
  
  if (num_tris == 0) {
    return YAOBI_NO_TRIANGLES;
  }
  
  if (m.tri_mesh->NumVertices() < 3) {
    return YAOBI_TOO_FEW_VERTS;
  }
  
  
  // check that some choice is set for the splitting point
  if (!(flags & YAOBI_TRIS_MEAN_SPLIT)   &&
      !(flags & YAOBI_TRIS_MEDIAN_SPLIT) &&
      !(flags & YAOBI_OBB_CENTER_SPLIT))
  {
    // default is mean split
    flags |= YAOBI_TRIS_MEAN_SPLIT;
  }
  
  // NOTE: This function does not touch the tri_mesh member
  m.Clear();
  
  m.obb_nodes        = new OBB_Node[2 * num_tris];
  m.num_obbs_alloced = 2 * num_tris;
  
  if (use_tri_nodes) {
    m.tri_nodes             = new TriNode[num_tris];
    m.num_tri_nodes_alloced = num_tris;
    m.num_tri_nodes         = 0;
  }
  
    
  parent.obb_indx = 0;
  parent.tris.reserve(num_tris);
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    parent.tris.push_back(i);
  }
  
  // compute first OBB
  compute_obb(m.obb_nodes[0], parent, *m.tri_mesh, flags);
  
  
  m.num_obbs = 1;
  
  build_stack.push(parent);
  
  
  while (!build_stack.empty()) {
    parent = build_stack.top();
    build_stack.pop();
    
    OBB_Node& parent_obb = m.obb_nodes[parent.obb_indx];
    
    if (use_tri_nodes && parent.tris.size() < 3) {
      // make a TriNode
      const int trinode_indx = m.num_tri_nodes;
      parent_obb.first_child = -(trinode_indx + 1);
      
      if (parent.tris.size() == 0) {
        m.tri_nodes[trinode_indx].tri1 = -1;
        m.tri_nodes[trinode_indx].tri2 = -1;
        printf("TriNode got zero triangles, should never happen!\n");
      } else if (parent.tris.size() == 1) {
        m.tri_nodes[trinode_indx].tri1 =  parent.tris[0];
        m.tri_nodes[trinode_indx].tri2 = -1;
      } else {
        m.tri_nodes[trinode_indx].tri1 =  parent.tris[0];
        m.tri_nodes[trinode_indx].tri2 =  parent.tris[1];
        
        // swap triangles?
        if (swap_tris) {
          if (tri_area_sqrd(*m.tri_mesh, parent.tris[0]) < tri_area_sqrd(*m.tri_mesh, parent.tris[1])) {
            if (!smallest_tri_first) {
              std::swap(m.tri_nodes[trinode_indx].tri1, m.tri_nodes[trinode_indx].tri2);
            }
          } else if (smallest_tri_first) {
            std::swap(m.tri_nodes[trinode_indx].tri1, m.tri_nodes[trinode_indx].tri2);
          }
        }
      }
      
      ++m.num_tri_nodes;
    } else if (parent.tris.size() > 1) {
      partition_triangles(parent, *m.tri_mesh, left_child.tris, right_child.tris);
      
      // compute the left child
      compute_obb(left_obb, left_child, *m.tri_mesh, flags);
      
      // compute the right child
      compute_obb(right_obb, right_child, *m.tri_mesh, flags);
      
      
      // setup the parent-child relations
      parent_obb.first_child = m.num_obbs;
      left_child.obb_indx    = m.num_obbs;
      right_child.obb_indx   = m.num_obbs + 1;
      
      // swap OBBs?
      if (swap_obbs) {
        if (left_obb.GetSize() < right_obb.GetSize()) {
          if (!smallest_obb_first) {
            std::swap(left_child.obb_indx, right_child.obb_indx);
          }
        } else if (smallest_obb_first) {
          std::swap(left_child.obb_indx, right_child.obb_indx);
        }
      }
      
      m.obb_nodes[left_child.obb_indx]  = left_obb;
      m.obb_nodes[right_child.obb_indx] = right_obb;
      
      // update the number of OBBs
      m.num_obbs += 2;
      
      // push both children on the stack
      build_stack.push(right_child);
      build_stack.push(left_child);
    } else {
      parent_obb.first_child = -(parent.tris[0] + 1);
    }
  }
  
  // release unused memory
  m.ShrinkToFit();
  
  // The collision model is now ready for use
  m.state |= CollModel::IS_VALID;
  
  return YAOBI_OK;
}

}  // namespace yaobi

//============================================================================


namespace {

using namespace yaobi;


void
partition_triangles(const BuildNode&        parent,
                    const TriMeshInterface& trim,
                    TriIndxVec&             left_child,
                    TriIndxVec&             right_child)
{
  double tri_verts[3][3];
  const TriIndxVec& tris      = parent.tris;
  const unsigned int num_tris = (unsigned int)tris.size();
  
  // make sure each child contains zero triangles
  left_child.clear();
  right_child.clear();
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    const int indx = tris[i];
    trim.GetTriangle(indx, tri_verts);
    
    // compute centroid of triangle
    VEC_PLUS_ASSIGN(tri_verts[0], tri_verts[1]);
    VEC_PLUS_ASSIGN(tri_verts[0], tri_verts[2]);
    
    // project onto axis
    const double x = DOT_PROD(tri_verts[0], parent.split_axis) / 3.0;
    
    if (x < parent.split_coord) {
      left_child.push_back(indx);
    } else {
      right_child.push_back(indx);
    }
  }
  
  if ((left_child.empty() || right_child.empty()) && num_tris > 1) {
    // do an arbitrary partitioning
    left_child.clear();
    right_child.clear();
    
    const unsigned int mid = num_tris / 2;
    left_child.insert(left_child.end(),   tris.begin(), tris.begin() + mid);
    right_child.insert(right_child.end(), tris.begin() + mid, tris.end());
  }
  
  return;
}

//============================================================================


void
compute_moments(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris, Moment& m_tot)
{
  // first collect all the moments, and obtain the area of the 
  // smallest nonzero area triangle.
  
  double a_min          = -1.0;
  bool degenerate_found = false;
  Moment* const m_tris  = new Moment[num_tris];
  double tri_verts[3][3];
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    trim.GetTriangle(indx_vec[i], tri_verts);
    compute_moments(tri_verts, m_tris[i]);
    
    if (m_tris[i].area <= 0.0) {
      degenerate_found = true;
    } else {
      if (a_min <= 0.0) {
        a_min = m_tris[i].area;
      } else if (m_tris[i].area < a_min) {
        a_min = m_tris[i].area;
      }
    }
  }
  
  if (degenerate_found) {
      /*
    fprintf(stderr, "----\n");
    fprintf(stderr, "Warning! Some triangle have zero area!\n");
    fprintf(stderr, "----\n");
      */

    // if there are any zero-area triangles, go back and set their area
    
    // if ALL the triangles have zero area, then set the area to 1.0
    if (a_min <= 0.0) { a_min = 1.0; }
    
    for (unsigned int i = 0; i < num_tris; ++i) {
      if (m_tris[i].area <= 0.0f) { m_tris[i].area = a_min; }
    }
  }
  
  clear_moments(m_tot);
  
  // now compute the moments for all triangles together
  for (unsigned int i = 0; i < num_tris; ++i) {
    accum_moments(m_tot, m_tris[i]);
  }
  
  // get correct mean by dividing with total area
  const double a_inv = 1.0 / m_tot.area;
  m_tot.mean[0] *= a_inv;
  m_tot.mean[1] *= a_inv;
  m_tot.mean[2] *= a_inv;
  
  // compute (scaled) covariance matrix
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      m_tot.cov[i][j] = m_tot.cov[i][j] - (m_tot.mean[i] * m_tot.mean[j]) * m_tot.area;
    }
  }
  
  //printf("accum mean: %f %f %f\n", m_tot.mean[0], m_tot.mean[1], m_tot.mean[2]); getchar();
  
  delete[] m_tris;
  
  return;
}

//============================================================================


void
compute_obb(OBB_Node&               obb,
            BuildNode&              bnode,
            const TriMeshInterface& trim,
            unsigned int            flags)
{
  Moment tris_mom;
  double e[3][3]; // eigen vectors
  double s[3];    // eigen values
  
  const int* const indx_vec   = &bnode.tris[0];
  const unsigned int num_tris = (unsigned int)bnode.tris.size();
  
  // compute moments for all triangles
  compute_moments(trim, indx_vec, num_tris, tris_mom);
  
  // compute eigen values for the covariance matrix
  eigen_3x3(e, s, tris_mom.cov);
  //printf("eigen values: %f  %f  %f\n", s[0], s[1], s[2]);
  //printf("eigen vectors:\n");
  //printf("%f  %f  %f\n%f  %f  %f\n%f  %f  %f\n",
  //  e[0][0], e[0][1], e[0][2],
  //  e[1][0], e[1][1], e[1][2],
  //  e[2][0], e[2][1], e[2][2]);
  //getchar();
  
  // sort the eigen vectors
  unsigned int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else             { min = 0; max = 1; }
  if (s[2] < s[min])      { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else                    { mid = 2; }
  
  
  obb.t_rel_top[R00] = static_cast<yaobi::Real>(e[0][max]);
  obb.t_rel_top[R10] = static_cast<yaobi::Real>(e[1][max]);
  obb.t_rel_top[R20] = static_cast<yaobi::Real>(e[2][max]);
  
  obb.t_rel_top[R01] = static_cast<yaobi::Real>(e[0][mid]);
  obb.t_rel_top[R11] = static_cast<yaobi::Real>(e[1][mid]);
  obb.t_rel_top[R21] = static_cast<yaobi::Real>(e[2][mid]);
  
  // compute the third column as the cross-product of the first two
  obb.t_rel_top[R02] = static_cast<yaobi::Real>(e[1][max]*e[2][mid] - e[2][max]*e[1][mid]);
  obb.t_rel_top[R12] = static_cast<yaobi::Real>(e[2][max]*e[0][mid] - e[0][max]*e[2][mid]);
  obb.t_rel_top[R22] = static_cast<yaobi::Real>(e[0][max]*e[1][mid] - e[1][max]*e[0][mid]);
  
  // fit the OBB to the triangles
  // this will set its dimensions and center
  Vector3 diam_vec;
  obb.FitToTriangles(trim, indx_vec, num_tris, diam_vec);
  
  obb.SortDimensions();
  
  // compute split-axis and split coordinate
  
  // Options:
  //  1) longest box dimension
  //  2) direction formed by the two most distant points
  if (flags & YAOBI_SPLIT_AXIS_FROM_DIAMETER) {
    const double len = sqrt(DOT_PROD(diam_vec, diam_vec));
    
    bnode.split_axis[0] = diam_vec[0] / len;
    bnode.split_axis[1] = diam_vec[1] / len;
    bnode.split_axis[2] = diam_vec[2] / len;
  } else {
    VEC_SET(bnode.split_axis, obb.t_rel_top[R00], obb.t_rel_top[R10], obb.t_rel_top[R20]);
  }
  
  // Options:
  //  1) mean of triangle centroids
  //  2) median of triangle centroids
  //  3) box center
  if (flags & YAOBI_TRIS_MEDIAN_SPLIT) {
    printf("median split!\n");
    bnode.split_coord = compute_median(bnode.split_axis, trim, bnode.tris);
  } else {
    bnode.split_coord = DOT_PROD(bnode.split_axis, tris_mom.mean);
  }
  
  return;
}

//============================================================================


double
tri_area_sqrd(const TriMeshInterface& trim, unsigned int tri_id)
{
  double tri_verts[3][3];
  double e1[3];
  double e2[3];
  double n[3];
  double a2;
  
  trim.GetTriangle(tri_id, tri_verts);
  
  VEC_SUB(e1, tri_verts[1], tri_verts[0]);
  VEC_SUB(e2, tri_verts[2], tri_verts[0]);
  CROSS_PROD(n, e1, e2);
  
  a2 = DOT_PROD(n, n);
  
  return fabs(a2);
}

//=============================================================================

/*
const OBB_Node
optimal_obb(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris)
{
  Vector3 tri_verts[3];
  Vector3 p;
  std::vector<bool> visited_vertices(trim.NumVertices(), false);
  
  // parameters affecting the performance of the algorithm
  const int grid_size   = 8;
  const int sample_size = 4000;
  
  // must begin with converting the triangle vertice to an array of gdiam_point
  const unsigned int num_points_max = MIN(3 * num_tris, trim.NumVertices());
  gdiam_real* const coords          = new gdiam_real[3 * num_points_max];
  
  unsigned int j = 0;
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    const IndexedTriangle tri(trim.GetIndexedTriangle(indx_vec[i]));
    
    if (!visited_vertices[tri.p1]) {
      visited_vertices[tri.p1] = true;
      trim.GetVertex(tri.p1, p);
      
      coords[j++] = (gdiam_real)p[0];
      coords[j++] = (gdiam_real)p[1];
      coords[j++] = (gdiam_real)p[2];
    }
    
    if (!visited_vertices[tri.p2]) {
      visited_vertices[tri.p2] = true;
      trim.GetVertex(tri.p2, p);
      
      coords[j++] = (gdiam_real)p[0];
      coords[j++] = (gdiam_real)p[1];
      coords[j++] = (gdiam_real)p[2];
    }
    
    if (!visited_vertices[tri.p3]) {
      visited_vertices[tri.p3] = true;
      trim.GetVertex(tri.p3, p);
      
      coords[j++] = (gdiam_real)p[0];
      coords[j++] = (gdiam_real)p[1];
      coords[j++] = (gdiam_real)p[2];
    }
  }
  
  OBB_Node obb;
  
  const unsigned int num_points     = j / 3;
  const unsigned int min_num_points = 12;
  
  if (num_points > min_num_points) {
    gdiam_point* const points = gdiam_convert(coords, num_points);
    
    gdiam_bbox bb = gdiam_approx_mvbb_grid_sample(points, num_points, grid_size, sample_size);
    
    delete[] points;
    
    SET_IDENTITY_TRANSFORM(obb.t_rel_top);
    
    obb.t_rel_top[R00] = bb.get_dir(0)[0];
    obb.t_rel_top[R10] = bb.get_dir(0)[1];
    obb.t_rel_top[R20] = bb.get_dir(0)[2];
    
    obb.t_rel_top[R01] = bb.get_dir(1)[0];
    obb.t_rel_top[R11] = bb.get_dir(1)[1];
    obb.t_rel_top[R21] = bb.get_dir(1)[2];
    
    obb.t_rel_top[R02] = bb.get_dir(2)[0];
    obb.t_rel_top[R12] = bb.get_dir(2)[1];
    obb.t_rel_top[R22] = bb.get_dir(2)[2];
  }
  
  delete[] coords;
  
  return obb;
}
*/

//============================================================================


Bool
is_planar(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris)
{
  double tri_verts[3][3];
  double e1[3];
  double e2[3];
  double n1[3];
  double n2[3];
  
  
  //static const double max_cos_angle = cos(0.1 * DEG_TO_RAD);
  const double max_cos_angle = 0.9;
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    trim.GetTriangle(indx_vec[i], tri_verts);
    
    
    VEC_SUB(e1, tri_verts[1], tri_verts[0]);
    VEC_SUB(e2, tri_verts[2], tri_verts[0]);
    CROSS_PROD(n1, e1, e2);
    
    double len = sqrt(DOT_PROD(n1, n1));
    
    if (len == 0.0f) continue;
    
    n1[0] /= len;
    n1[1] /= len;
    n1[2] /= len;
    
    for (unsigned int j = i + 1; j < num_tris; ++j) {
      trim.GetTriangle(indx_vec[j], tri_verts);
      
      VEC_SUB(e1, tri_verts[1], tri_verts[0]);
      VEC_SUB(e2, tri_verts[2], tri_verts[0]);
      CROSS_PROD(n2, e1, e2);
      
      len = sqrt(DOT_PROD(n2, n2));
      
      if (len == 0.0f) continue;
      
      n2[0] /= len;
      n2[1] /= len;
      n2[2] /= len;
      
      const double cos_angle = DOT_PROD(n1, n2);
      
      if (fabs(cos_angle) < max_cos_angle) { return FALSE; }
    }
  }
  
  return TRUE;
}

//============================================================================


bool
optimal_obb_from_tris(OBB_Node& obb, const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris)
{
  OBB_Node tmp;
  double tri_verts[3][3];
  double e1[3];
  double e2[3];
  double e3[3];
  double n1[3];
  double n2[3];
  double n3[3];
  double min_size      = 1.0e10;
  bool valid_obb_found = false;
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    trim.GetTriangle(indx_vec[i], tri_verts);
    
    VEC_SUB(e1, tri_verts[1], tri_verts[0]);
    VEC_SUB(e2, tri_verts[2], tri_verts[0]);
    CROSS_PROD(n1, e1, e2);
    
    double len = sqrt(DOT_PROD(n1, n1));
    
    if (len == 0.0) continue;
    
    n1[0] /= len;
    n1[1] /= len;
    n1[2] /= len;
    
    // loop over all edges
    for (unsigned int j = 0; j < num_tris; ++j) {
      trim.GetTriangle(indx_vec[j], tri_verts);
      
      VEC_SUB(e1, tri_verts[1], tri_verts[0]);
      VEC_SUB(e2, tri_verts[2], tri_verts[1]);
      VEC_SUB(e3, tri_verts[0], tri_verts[2]);
      
      // try edge e1
      CROSS_PROD(n2, n1, e1);
      
      len = sqrt(DOT_PROD(n2, n2));
      if (len != 0.0) {
        n2[0] /= len;
        n2[1] /= len;
        n2[2] /= len;
        
        CROSS_PROD(n3, n1, n2);
        
        tmp.t_rel_top[R00] = static_cast<yaobi::Real>(n1[0]);
        tmp.t_rel_top[R10] = static_cast<yaobi::Real>(n1[1]);
        tmp.t_rel_top[R20] = static_cast<yaobi::Real>(n1[2]);
        
        tmp.t_rel_top[R01] = static_cast<yaobi::Real>(n2[0]);
        tmp.t_rel_top[R11] = static_cast<yaobi::Real>(n2[1]);
        tmp.t_rel_top[R21] = static_cast<yaobi::Real>(n2[2]);
        
        tmp.t_rel_top[R02] = static_cast<yaobi::Real>(n3[0]);
        tmp.t_rel_top[R12] = static_cast<yaobi::Real>(n3[1]);
        tmp.t_rel_top[R22] = static_cast<yaobi::Real>(n3[2]);
        
        tmp.FitToTriangles(trim, indx_vec, num_tris);
        
        if (tmp.GetSqrdDiag() < min_size) {
          obb             = tmp;
          min_size        = tmp.GetSqrdDiag();
          valid_obb_found = true;
        }
      }
      
      // try edge e2
      CROSS_PROD(n2, n1, e2);
      
      len = sqrt(DOT_PROD(n2, n2));
      if (len != 0.0) {
        n2[0] /= len;
        n2[1] /= len;
        n2[2] /= len;
        
        CROSS_PROD(n3, n1, n2);
        
        tmp.t_rel_top[R00] = static_cast<yaobi::Real>(n1[0]);
        tmp.t_rel_top[R10] = static_cast<yaobi::Real>(n1[1]);
        tmp.t_rel_top[R20] = static_cast<yaobi::Real>(n1[2]);
        
        tmp.t_rel_top[R01] = static_cast<yaobi::Real>(n2[0]);
        tmp.t_rel_top[R11] = static_cast<yaobi::Real>(n2[1]);
        tmp.t_rel_top[R21] = static_cast<yaobi::Real>(n2[2]);
        
        tmp.t_rel_top[R02] = static_cast<yaobi::Real>(n3[0]);
        tmp.t_rel_top[R12] = static_cast<yaobi::Real>(n3[1]);
        tmp.t_rel_top[R22] = static_cast<yaobi::Real>(n3[2]);
        
        tmp.FitToTriangles(trim, indx_vec, num_tris);
        
        if (tmp.GetSqrdDiag() < min_size) {
          obb             = tmp;
          min_size        = tmp.GetSqrdDiag();
          valid_obb_found = true;
        }
      }
      
      // try edge e3
      CROSS_PROD(n2, n1, e3);
      
      len = sqrt(DOT_PROD(n2, n2));
      if (len != 0.0) {
        n2[0] /= len;
        n2[1] /= len;
        n2[2] /= len;
        
        CROSS_PROD(n3, n1, n2);
        
        tmp.t_rel_top[R00] = static_cast<yaobi::Real>(n1[0]);
        tmp.t_rel_top[R10] = static_cast<yaobi::Real>(n1[1]);
        tmp.t_rel_top[R20] = static_cast<yaobi::Real>(n1[2]);
        
        tmp.t_rel_top[R01] = static_cast<yaobi::Real>(n2[0]);
        tmp.t_rel_top[R11] = static_cast<yaobi::Real>(n2[1]);
        tmp.t_rel_top[R21] = static_cast<yaobi::Real>(n2[2]);
        
        tmp.t_rel_top[R02] = static_cast<yaobi::Real>(n3[0]);
        tmp.t_rel_top[R12] = static_cast<yaobi::Real>(n3[1]);
        tmp.t_rel_top[R22] = static_cast<yaobi::Real>(n3[2]);
        
        tmp.FitToTriangles(trim, indx_vec, num_tris);
        
        if (tmp.GetSqrdDiag() < min_size) {
          obb             = tmp;
          min_size        = tmp.GetSqrdDiag();
          valid_obb_found = true;
        }
      }
    }
  }
  
  if (valid_obb_found) {
    // make sure the smallest dimension is in the z-direction
    obb.SortDimensions();
  }
  
  return valid_obb_found;
}

//============================================================================


double
compute_median(const double axis[3], const TriMeshInterface& trim, const TriIndxVec& tris)
{
  assert(!tris.empty());
  
  double tri_verts[3][3];
  const unsigned int num_tris = (unsigned int) tris.size();
  std::vector<double> projs(num_tris);
  
  for (unsigned int i = 0; i < num_tris; ++i) {
    trim.GetTriangle(tris[i], tri_verts);
    
    // compute centroid of triangle
    VEC_PLUS_ASSIGN(tri_verts[0], tri_verts[1]);
    VEC_PLUS_ASSIGN(tri_verts[0], tri_verts[2]);
    
    // project onto axis
    projs[i] = DOT_PROD(tri_verts[0], axis) / 3.0;
  }
  
  std::sort(projs.begin(), projs.end());
  
  double median;
  
  if (num_tris & 1) {
    median = projs[num_tris / 2];
  } else {
    median = (projs[(num_tris / 2) - 1] + projs[num_tris / 2]) / 2.0;
  }
  
  return median;
}

} // anonymous namespace
