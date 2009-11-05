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

#ifndef YAOBI_FPU_H_
#define YAOBI_FPU_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_fpu.h
//! \brief Optimized math functions
//! \note These optimizations only apply if Yaobi is using float.
//!


#include "yaobi_settings.h"

namespace yaobi {

//! Returns the minimum of \a a and \a b
#define MIN(a,b) (((a) > (b))? (b) : (a))

//! Returns the maximum of \a a and \a b
#define MAX(a,b) (((a) > (b))? (a) : (b))

#ifdef YAOBI_USE_FLOAT
  #define SIGN_BITMASK            0x80000000
  
  //! Integer representation of a floating-point value
  #define IR(x)                   ((unsigned int&)(x))
  
  //! Signed integer representation of a floating-point value
  #define SIR(x)                  ((int&)(x))
  
  //! Absolute integer representation of a floating-point value
  #define AIR(x)                  (IR(x) & 0x7fffffff)
  
  //! Floating-point representation of an integer value
  #define FR(x)                   ((float&)(x))
  
  //! Integer-based comparison of a floating point value.
  //! Don't use it blindly, it can be faster or slower than the FPU comparison, depends on the context.
  #define IS_NEGATIVE_FLOAT(x)    (IR(x) & SIGN_BITMASK)
  
#ifdef YAOBI_USE_FAST_FABS
    //! Fast fabs for floating-point values. It just clears the sign bit.
    //! This is the floating point optimization that has the largest impact on the speed of Yaobi.
    //! \note From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
    YAOBI_INLINE float FastFabs(float x)
    {
      const unsigned int float_bits = IR(x) & 0x7fffffff;
      return FR(float_bits);
    }
    
    #define FABS(x) FastFabs(x)
#else
    #define FABS(x) (((x) < 0.0f)? -(x) : (x))
#endif // YAOBI_USE_FAST_FABS
  
  
  // NOTE: From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
  #ifdef YAOBI_USE_FCOMI
    #define FCOMI_ST0   __asm    _emit   0xdb    __asm    _emit   0xf0
    #define FCOMIP_ST0  __asm    _emit   0xdf    __asm    _emit   0xf0
    #define FCMOVB_ST0  __asm    _emit   0xda    __asm    _emit   0xc0
    #define FCMOVNB_ST0 __asm    _emit   0xdb    __asm    _emit   0xc0
    
    #define FCOMI_ST1   __asm    _emit   0xdb    __asm    _emit   0xf1
    #define FCOMIP_ST1  __asm    _emit   0xdf    __asm    _emit   0xf1
    #define FCMOVB_ST1  __asm    _emit   0xda    __asm    _emit   0xc1
    #define FCMOVNB_ST1 __asm    _emit   0xdb    __asm    _emit   0xc1
    
    #define FCOMI_ST2   __asm    _emit   0xdb    __asm    _emit   0xf2
    #define FCOMIP_ST2  __asm    _emit   0xdf    __asm    _emit   0xf2
    #define FCMOVB_ST2  __asm    _emit   0xda    __asm    _emit   0xc2
    #define FCMOVNB_ST2 __asm    _emit   0xdb    __asm    _emit   0xc2
    
    #define FCOMI_ST3   __asm    _emit   0xdb    __asm    _emit   0xf3
    #define FCOMIP_ST3  __asm    _emit   0xdf    __asm    _emit   0xf3
    #define FCMOVB_ST3  __asm    _emit   0xda    __asm    _emit   0xc3
    #define FCMOVNB_ST3 __asm    _emit   0xdb    __asm    _emit   0xc3
    
    #define FCOMI_ST4   __asm    _emit   0xdb    __asm    _emit   0xf4
    #define FCOMIP_ST4  __asm    _emit   0xdf    __asm    _emit   0xf4
    #define FCMOVB_ST4  __asm    _emit   0xda    __asm    _emit   0xc4
    #define FCMOVNB_ST4 __asm    _emit   0xdb    __asm    _emit   0xc4
    
    #define FCOMI_ST5   __asm    _emit   0xdb    __asm    _emit   0xf5
    #define FCOMIP_ST5  __asm    _emit   0xdf    __asm    _emit   0xf5
    #define FCMOVB_ST5  __asm    _emit   0xda    __asm    _emit   0xc5
    #define FCMOVNB_ST5 __asm    _emit   0xdb    __asm    _emit   0xc5
    
    #define FCOMI_ST6   __asm    _emit   0xdb    __asm    _emit   0xf6
    #define FCOMIP_ST6  __asm    _emit   0xdf    __asm    _emit   0xf6
    #define FCMOVB_ST6  __asm    _emit   0xda    __asm    _emit   0xc6
    #define FCMOVNB_ST6 __asm    _emit   0xdb    __asm    _emit   0xc6
    
    #define FCOMI_ST7   __asm    _emit   0xdb    __asm    _emit   0xf7
    #define FCOMIP_ST7  __asm    _emit   0xdf    __asm    _emit   0xf7
    #define FCMOVB_ST7  __asm    _emit   0xda    __asm    _emit   0xc7
    #define FCMOVNB_ST7 __asm    _emit   0xdb    __asm    _emit   0xc7
    
    //! A function to find MAX(a,b) using FCOMI/FCMOV
    //! \note From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
    YAOBI_INLINE float
    FCMax2(float a, float b)
    {
      float Res;
      __asm    fld     [a]
      __asm    fld     [b]
      FCOMI_ST1
      FCMOVB_ST1
      __asm    fstp    [Res]
      __asm    fcomp
      return Res;
    }
    
    //! A function to find MIN(a,b) using FCOMI/FCMOV
    //! \note From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
    YAOBI_INLINE float
    FCMin2(float a, float b)
    {
      float Res;
      __asm    fld     [a]
      __asm    fld     [b]
      FCOMI_ST1
      FCMOVNB_ST1
      __asm    fstp    [Res]
      __asm    fcomp
      return Res;
    }
    
    //! A function to find MAX(a,b,c) using FCOMI/FCMOV
    //! \note From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
    YAOBI_INLINE float
    FCMax3(float a, float b, float c)
    {
      float Res;
      __asm    fld     [a]
      __asm    fld     [b]
      __asm    fld     [c]
      FCOMI_ST1
      FCMOVB_ST1
      FCOMI_ST2
      FCMOVB_ST2
      __asm    fstp    [Res]
      __asm    fcompp
      return Res;
    }
    
    //! A function to find MIN(a,b,c) using FCOMI/FCMOV
    //! \note From Pierre Terdiman's Opcode library, http://www.codercorner.com/Opcode.htm
    YAOBI_INLINE float
    FCMin3(float a, float b, float c)
    {
      float Res;
      __asm    fld     [a]
      __asm    fld     [b]
      __asm    fld     [c]
      FCOMI_ST1
      FCMOVNB_ST1
      FCOMI_ST2
      FCMOVNB_ST2
      __asm    fstp    [Res]
      __asm    fcompp
      return Res;
    }
    
    #define MIN2(a,b)   FCMin2(a,b)
    #define MAX2(a,b)   FCMax2(a,b)
    
    //! Returns the minimum of the three values \a a, \a b, and \a c
    #define MIN3(a,b,c) FCMin3(a,b,c)
    
    //! Returns the maximum of the three values \a a, \a b, and \a c
    #define MAX3(a,b,c) FCMax3(a,b,c)
  #else
    #define MIN2(a,b)   MIN(a,b)
    #define MAX2(a,b)   MAX(a,b)
    
    YAOBI_INLINE float
    MIN3(float a, float b, float c)
    {
      const float t = MIN2(a, b);
      return MIN2(t, c);
    }
    
    YAOBI_INLINE float
    MAX3(float a, float b, float c)
    {
      const float t = MAX2(a, b);
      return MAX2(t, c);
    }
  #endif // YAOBI_USE_FCOMI
  
#else
  #define FABS(x) (((x) < 0.0)? -(x) : (x))
  
  #define MIN2(a,b)   MIN(a,b)
  #define MAX2(a,b)   MAX(a,b)
  
  YAOBI_INLINE double
  MIN3(double a, double b, double c)
  {
    const double t = MIN2(a, b);
    return MIN2(t, c);
  }
  
  YAOBI_INLINE double
  MAX3(double a, double b, double c)
  {
    const double t = MAX2(a, b);
    return MAX2(t, c);
  }
  
#endif // YAOBI_USE_FLOAT

} // namespace yaobi

#endif // YAOBI_FPU_H_
