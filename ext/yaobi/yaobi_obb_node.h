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

#ifndef YAOBI_OBB_NODE_H_
#define YAOBI_OBB_NODE_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_obb_node.h
//!
//! \brief OBB_Node and TriNode declarations
//!
//! An OBB_Node defines an oriented bounding box that contains either other
//! OBB_Nodes, or TriNodes.
//! \see TriNode
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#include "yaobi_settings.h"
#include "yaobi_vector.h"

namespace yaobi {

class TriMeshInterface;

//! \brief Represents an oriented bounding box.
//!
//! An OBB_Node contains either other OBB_Nodes, or TriNodes.
//!
//! \see TriNode
struct OBB_Node {
  OBB_Node();
  
  //! Returns TRUE if the OBB_Node is a leaf-node, that is, it does not contain other
  //! OBB_Node objects.
  Bool IsLeaf() const;
  
  //! \return The 'size' of the OBB.
  //!
  //! Different measures can be used to give a measure of
  //! the OBB size. Examples are: volume, surface area, and box diagonal.
  //! Experiments have shown that the surface area is the most efficient
  //! size measure.
  Real GetSize() const;
  
  //! \return The volume of the OBB
  Real GetVolume() const;
  
  //! \return The squared diagonal of the OBB
  Real GetSqrdDiag() const;
  
  //! Using the orientation of the OBB, this function computes the dimension
  //! and center of the OBB.
  //! When returning, \a diam_vec will contain an approximation of the vector between the two most
  //! distant vertices. It is OK to set this pointer to null.
  void FitToTriangles(const TriMeshInterface& trim, const int indx_vec[], unsigned int num_tris, Real diam_vec[3] = 0);
  
  //! Reorients the box axes so that \a dim_x >= \a dim_y >= \a dim_z
  void SortDimensions();
  
  //! The pose relative the topmost object in the hierarchy
  Transform t_rel_top;
  
  //! Half-dimensions of the OBB
  Real dim[3];
  
  //! \brief Index to the first child (an OBB_Node or a TriNode)
  //!
  //! A positive value is the index of the first child of this node,
  //! a negative value is -(index + 1) of a TriNode
  int first_child;
};

//! \brief A TriNode contains one or two triangles.
//!
//! If index TriNode::tri2 is negative, then
//! the node contains only one triangle.
struct TriNode {
  int tri1; //!< Index to first triangle
  int tri2; //!< A negative index means this node contains only one triangle
};

//////////////////////////////////////////////////////////////////////////////
/////////////////          Inline definitions below          /////////////////


inline
OBB_Node::OBB_Node():
first_child(0)
{
}

//============================================================================


inline Bool
OBB_Node::IsLeaf() const
{
  return first_child < 0;
}

//============================================================================


YAOBI_INLINE Real
OBB_Node::GetSize() const
{
  //return (dim[0]*dim[0] + dim[1]*dim[1] + dim[2]*dim[2]); // squared diagonal
  //return dim[0]*dim[0] + dim[1]*dim[1];                   // largest face diagonal
  return dim[0]*dim[1] + dim[1]*dim[2] + dim[2]*dim[0];     // surface area
}

//============================================================================


inline Real
OBB_Node::GetVolume() const
{
  return 8.0f * dim[0] * dim[1] * dim[2];
}

//============================================================================


inline Real
OBB_Node::GetSqrdDiag() const
{
  return dim[0]*dim[0] + dim[1]*dim[1] + dim[2]*dim[2];
}


} // namespace yaobi

#endif // YAOBI_OBB_NODE_H_
