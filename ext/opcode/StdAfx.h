///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *	OPCODE - Optimized Collision Detection
 *	Copyright (C) 2001 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/Opcode.htm
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_STDAFX_H__EFB95044_1D31_11D5_8B0F_0050BAC83302__INCLUDED_)
#define AFX_STDAFX_H__EFB95044_1D31_11D5_8B0F_0050BAC83302__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <float.h>
#include <math.h>

#ifndef ASSERT
    #define	ASSERT	assert
#endif

//#define	Log
#define	SetIceError(a,b)	false
#define	EC_OUTOFMEMORY	"Out of memory"
#define	Alignment


// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

// Constants
#define PI					3.1415926535897932384626433832795028841971693993751f	//!<	PI
#define HALFPI				1.57079632679489661923f									//!<	0.5 * PI
#define TWOPI				6.28318530717958647692f									//!<	2.0 * PI
#define INVPI				0.31830988618379067154f									//!<	1.0 / PI

#define RADTODEG			57.2957795130823208768f									//!<	180.0 / PI, convert radians to degrees
#define DEGTORAD			0.01745329251994329577f									//!<	PI / 180.0, convert degrees to radians

#define EXP					2.71828182845904523536f									//!<	e
#define INVLOG2				3.32192809488736234787f									//!<	1.0 / log10(2)
#define LN2					0.693147180559945f										//!<	ln(2)
#define	INVLN2				1.44269504089f											//!<	1.0f / ln(2)

#define INV3				0.33333333333333333333f									//!<	1/3
#define INV6				0.16666666666666666666f									//!<	1/6
#define INV7				0.14285714285714285714f									//!<	1/7
#define INV9				0.11111111111111111111f									//!<	1/9
#define INV255				0.00392156862745098039f									//!<	1/255

#define null				0														//!<	our own NULL pointer

#define inline_ inline
//#define ICEMATHS_API

//define opcode types
    // New types
typedef signed char			sbyte;		//!<	sizeof(sbyte)	must be 1
typedef unsigned char		ubyte;		//!<	sizeof(ubyte)	must be 1
typedef signed short		sword;		//!<	sizeof(sword)	must be 2
typedef unsigned short		uword;		//!<	sizeof(uword)	must be 2
typedef signed int			sdword;		//!<	sizeof(sdword)	must be 4
typedef unsigned int		udword;		//!<	sizeof(udword)	must be 4

typedef signed long long		sqword;		//!<	sizeof(sqword)	must be 8
typedef unsigned long long	uqword;		//!<	sizeof(uqword)	must be 8

typedef float				float32;	//!<	sizeof(float32)	must be 4
typedef double				float64;	//!<	sizeof(float64)	must be 4
typedef int					BOOL;

#define FALSE false
#define TRUE true

    #define	SQRT2				1.41421356237f											//!< sqrt(2)
    #define	INVSQRT2			0.707106781188f											//!< 1 / sqrt(2)

    #define	SQRT3				1.73205080757f											//!< sqrt(3)
    #define	INVSQRT3			0.577350269189f											//!< 1 / sqrt(3)


    typedef udword				DynID;		//!< Dynamic identifier
#ifdef USE_HANDLE_MANAGER
    typedef udword				KID;		//!< Kernel ID
//	DECLARE_ICE_HANDLE(KID);
#else
    typedef uword				KID;		//!< Kernel ID
#endif
    typedef udword				RTYPE;		//!< Relationship-type (!) between owners and references
    #define	INVALID_ID			0xffffffff	//!< Invalid dword ID (counterpart of null pointers)
#ifdef USE_HANDLE_MANAGER
    #define	INVALID_KID			0xffffffff	//!< Invalid Kernel ID
#else
    #define	INVALID_KID			0xffff		//!< Invalid Kernel ID
#endif
    #define	INVALID_NUMBER		0xDEADBEEF	//!< Standard junk value



// Type ranges
#define	MAX_SBYTE			0x7f							//!<	max possible sbyte value
#define	MIN_SBYTE			0x80							//!<	min possible sbyte value
#define	MAX_UBYTE			0xff							//!<	max possible ubyte value
#define	MIN_UBYTE			0x00							//!<	min possible ubyte value
#define	MAX_SWORD			0x7fff							//!<	max possible sword value
#define	MIN_SWORD			0x8000							//!<	min possible sword value
#define	MAX_UWORD			0xffff							//!<	max possible uword value
#define	MIN_UWORD			0x0000							//!<	min possible uword value
#define	MAX_SDWORD			0x7fffffff						//!<	max possible sdword value
#define	MIN_SDWORD			0x80000000						//!<	min possible sdword value
#define	MAX_UDWORD			0xffffffff						//!<	max possible udword value
#define	MIN_UDWORD			0x00000000						//!<	min possible udword value
#define	MAX_FLOAT			FLT_MAX							//!<	max possible float value
#define	MIN_FLOAT			(-FLT_MAX)						//!<	min possible loat value
#define IEEE_1_0			0x3f800000						//!<	integer representation of 1.0
#define IEEE_255_0			0x437f0000						//!<	integer representation of 255.0
#define IEEE_MAX_FLOAT		0x7f7fffff						//!<	integer representation of MAX_FLOAT
#define IEEE_MIN_FLOAT		0xff7fffff						//!<	integer representation of MIN_FLOAT

#define ONE_OVER_RAND_MAX	(1.0f / float(RAND_MAX))		//!<	Inverse of the max possible value returned by rand()

#undef		MIN
#undef		MAX
#define		MIN(a, b)       ((a) < (b) ? (a) : (b))			//!<	Returns the min value between a and b
#define		MAX(a, b)       ((a) > (b) ? (a) : (b))			//!<	Returns the max value between a and b
#define		MAXMAX(a,b,c)   ((a) > (b) ? MAX (a,c) : MAX (b,c))	//!<	Returns the max value between a, b and c

template<class T>	inline_ const T&	TMin	(const T& a, const T& b)	{ return b < a ? b : a;	}
template<class T>	inline_ const T&	TMax	(const T& a, const T& b)	{ return a < b ? b : a;	}
template<class T>	inline_ void		TSetMin	(T& a, const T& b)			{ if(a>b)	a = b;		}
template<class T>	inline_ void		TSetMax	(T& a, const T& b)			{ if(a<b)	a = b;		}

#define		SQR(x)			((x)*(x))						//!<	Returns x square
#define		CUBE(x)			((x)*(x)*(x))					//!<	Returns x cube

#define		AND		&										//!<	...
#define		OR		|										//!<	...
#define		XOR		^										//!<	...

#define		QUADRAT(x)		((x)*(x))						//!<	Returns x square

#include "FPU_port.h"
#include "MEMMACRO_port.h"

#define FUNCTION				extern "C"

// Cosmetic stuff [mainly useful with multiple inheritance]
#define	override(baseclass)	virtual

#include "opcode_port.h"

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_STDAFX_H__EFB95044_1D31_11D5_8B0F_0050BAC83302__INCLUDED_)
