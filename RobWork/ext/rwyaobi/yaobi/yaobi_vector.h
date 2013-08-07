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

#ifndef YAOBI_VECTOR_H_
#define YAOBI_VECTOR_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_vector.h
//! \brief Simple typedefs for vectors and matrices
//!
//! These typedefs make it easy to switch to SSE2 instructions, if they should
//! prove to be more efficient.
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#include "yaobi_settings.h"

namespace yaobi {

#ifdef YAOBI_USE_SSE2
  //! SSE2 instructions run faster if their arguments are properly aligned
  //! \note Results so far have not been good.
  typedef __declspec(align(16)) float Transform[16]; //!< Used as transformation matrices
  typedef __declspec(align(16)) float Vector3[4];    //!< Used as 3D vectors
#else
  typedef Real Transform[12]; //!< Used as transformation matrices
  typedef Real Vector3[3];    //!< Used as 3D vectors
#endif


} // namespace yaobi

#endif // YAOBI_VECTOR_H_
