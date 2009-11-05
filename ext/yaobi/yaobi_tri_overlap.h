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

#ifndef YAOBI_TRI_OVERLAP_H_
#define YAOBI_TRI_OVERLAP_H_

#include "yaobi_settings.h"
#include "yaobi_vector.h"

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_tri_overlap.h
//! \brief A function for determining whether two 3D triangles overlap.
//!
//! \note Users can at compile-time choose between three different algorithms.
//!

namespace yaobi {

//! \param[in] t1 Three triangle vertices
//! \param[in] t2 Three triangle vertices
//! \return TRUE if the triangles overlap.
Bool tri_tri_overlap_3d(const Vector3 t1[], const Vector3 t2[]);

//! returns the 'name' of the algorithm used for the triangle overlap test
//! The possible algorithms are:
//!   "PQP overlap test"
//!   "Moller's overlap test"
//!   "Guigue's overlap test"
const char* tri_overlap_algorithm();

} // namespace yaobi


#endif // YAOBI_TRI_OVERLAP_H_
