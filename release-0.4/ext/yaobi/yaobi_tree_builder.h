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

#ifndef YAOBI_TREE_BUILDER_H_
#define YAOBI_TREE_BUILDER_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_tree_builder.h
//! \brief Interface for building OBB-trees
//!
//! The interface provides numerous settings that control how the tree is built.
//! The default settings should be the fastest for most cases.
//!
//! \todo Provide a method for bottom-up construction of the tree.
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#include "yaobi_settings.h"

namespace yaobi {

//! forward declaration
class CollModel;

//! The leaf nodes in the OBB-tree will be \a TriNode objects instead of \a OBB_Node objects.
//! A \a TriNode will contain one or two triangles. Using \a TriNodes will roughly half the
//! memory consumption of the OBB-tree. As an extra plus, collision queries run faster.
#define YAOBI_USE_TRI_NODES            (1U << 0)

//! The 'left' and 'right' triangles of a \a TriNode are sorted such that the one
//! with the largest area comes first. If we are only interested in the first contact, then it
//! is usually faster to have the largest triangle first.
#define YAOBI_LARGEST_TRI_FIRST        (1U << 1)

//! The 'left' and 'right' triangles of a \a TriNode are sorted such that the one
//! with the smallest area comes first.
#define YAOBI_SMALLEST_TRI_FIRST       (1U << 2)

//! The 'left' and 'right' children of an \a OBB_Node are sorted such that the
//! largest child comes first. The meaning of OBB size is defined by the function
//! \a OBB_Node::GetSize. If we are only interested in the first contact, then it
//! is usually faster to have the largest OBB first.
#define YAOBI_LARGEST_OBB_FIRST        (1U << 3)

//! The 'left' and 'right' children of an \a OBB_Node are sorted such that the
//! smallest child comes first. The meaning of OBB size is defined by the function
//! \a OBB_Node::GetSize
#define YAOBI_SMALLEST_OBB_FIRST       (1U << 4)

//! When an OBB is split, the split coordinate is computed from the mean value of the
//! (area weighted) triangle centroids.
#define YAOBI_TRIS_MEAN_SPLIT          (1U << 5)

//! When an OBB is split, the split coordinate is computed from the median of the
//! projection the triangle centroids onto the split axis.
#define YAOBI_TRIS_MEDIAN_SPLIT        (1U << 6)

//! When an OBB is split, the split coordinate is determined by the projection of
//! the OBB center onto the split axis.
#define YAOBI_OBB_CENTER_SPLIT         (1U << 7)

//! The split axis is determined by the vector between the most distant points in the OBB
#define YAOBI_SPLIT_AXIS_FROM_DIAMETER (1U << 8)

//! The default settings should give the fastest OBB-trees for most cases.
#define YAOBI_DEF_BUILD_FLAG (YAOBI_USE_TRI_NODES | YAOBI_LARGEST_TRI_FIRST | YAOBI_LARGEST_OBB_FIRST | YAOBI_TRIS_MEAN_SPLIT)

//! \brief Build the OBB-tree with the given settings
//!
//! \param[in,out] m     A collision model
//! \param[in]     flags Flags controlling the process of building the OBB-tree
//! \return YAOBI_OK if successful
int build_obb_tree(CollModel& m, unsigned int flags = YAOBI_DEF_BUILD_FLAG);

} // namespace yaobi

#endif // YAOBI_TREE_BUILDER_H_
