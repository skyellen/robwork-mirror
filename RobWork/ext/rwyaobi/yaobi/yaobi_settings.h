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

#ifndef YAOBI_SETTINGS_H_
#define YAOBI_SETTINGS_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_settings.h
//! \brief Contains all the compile-time settings for Yaobi.
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#ifndef _MSC_VER
#include "yaobi_config.h"
#else

//! Yaobi runs slightly faster with single-precision and uses less memory.
//! Can run up to 1.6 times faster with all the 'float options' turned on.
#define YAOBI_USE_FLOAT

//! Tests have shown that using native bool costs a *few* cycles extra. (Tested with
//! Microsoft Visual C++ .NET)
//#define YAOBI_USE_NATIVE_BOOL

//! Remove this if your application uses float to store mesh data
//#define YAOBI_APP_USE_DOUBLE

//#define YAOBI_VERT_INDEX_SHORT

//! Use epsilon value in tri-tri overlap test to increase robustness (at the expense of
//! a few extra if-statements in the triangle overlap test)
//#define YAOBI_TRITRI_EPSILON_TEST

//! Tests have shown that the last nine tests in the separating axis theorem rarely find any
//! separated OBB's. Therefore it is often more efficient to skip them completely.
//#define YAOBI_FULL_OBB_TEST

// NOTE: The following settings only apply if single-precision is used
#ifdef YAOBI_USE_FLOAT
   
   //! Fast fabs for floating-point values. It just clears the sign bit.
   //! This is the floating point optimization that has the largest impact, at
   //! least using Visual Studio 2003
   //! \note Has no effect if double-precision is used, that is, if YAOBI_USE_FLOAT is not defined.
#  define YAOBI_USE_FAST_FABS
   
   //! Use FCOMI / FCMOV on Pentium-Pro based processors (comment that line to use plain C++)
   //! \note Has no effect if double-precision is used, that is, if YAOBI_USE_FLOAT is not defined.
#ifndef MSVC_AMD64
#  define YAOBI_USE_FCOMI
#endif
   
   //! Interpret floating point numbers as integers to optimize comparisons of the following type:
   //!    |x| > y
   //! \note Has no effect if double-precision is used, that is, if YAOBI_USE_FLOAT is not defined.
#  define YAOBI_CPU_COMPARE
   
   //! Will use SSE2 instructions for matrix and vector multiplication
   //! \note Not fully implemented yet.
   //! \note Has no effect if double-precision is used, that is, if YAOBI_USE_FLOAT is not defined.
//#  define YAOBI_USE_SSE2
#endif // YAOBI_USE_FLOAT

#endif //HAVE_CONFIG_H 

#ifdef _MSC_VER
  //! \todo Check more compiler specifics here
  //!
  //! Even though some functions are very beneficial for inlining, an ordinary
  //! 'inline' is sometimes not enough for an obnoxious compiler. Therefore we
  //! define our own stronger inline-directive, which of course must be compiler
  //! dependent.
#  define YAOBI_INLINE __forceinline
#else
#  define YAOBI_INLINE inline
#endif

//============================================================================

namespace yaobi {

//! The floating-point type used internally by Yaobi
#ifdef YAOBI_USE_FLOAT
  typedef float Real;
#else
  typedef double Real;
#endif

//! The floating-point type that the application uses for storing mesh data.
//!
//! \note If not the same as the type used by Yaobi, then a cast will be
//! applied for every coordinate access.
#ifdef YAOBI_APP_USE_DOUBLE
  typedef double AppRealT;
#else
  typedef Real AppRealT;
#endif

//! The integer type used for indexing vertex arrays
#ifdef YAOBI_VERT_INDEX_SHORT
  typedef unsigned short VertIndex;
#else
  typedef unsigned int VertIndex;
#endif

//! The boolean type used by Yaobi. Tests have shown that using the native bool
//! costs a \a few cycles extra.
#ifdef YAOBI_USE_NATIVE_BOOL
   typedef bool Bool;
#  define TRUE true
#  define FALSE false
#else
  typedef unsigned int Bool;
#  ifndef TRUE
#    define TRUE  1
#  endif

#  ifndef FALSE
#    define FALSE 0
#  endif
#endif // YAOBI_USE_NATIVE_BOOL


//! \brief Determines whether mesh data is shared with the application or
//! owned by Yaobi.
//! \see TriMeshInterface, CollModel
enum ShareMode {
  //! shared data will not be deleted by Yaobi
  SHARE_DATA,
  
  //! (mesh) data will be deleted by a Yaobi destructor
  OWN_DATA
};

} // namespace yaobi

#endif // YAOBI_SETTINGS_H_
