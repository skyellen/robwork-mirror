
// Should be included by Opcode.h if needed

    #define ICE_DONT_CHECK_COMPILER_OPTIONS

    // From Windows...
    typedef int                 BOOL;
    #ifndef FALSE
    #define FALSE               0
    #endif

    #ifndef TRUE
    #define TRUE                1
    #endif

    #include <stdio.h>
    #include <stdlib.h>
    #include <assert.h>
    #include <string.h>
    #include <float.h>
    #include <math.h>

    #ifndef ASSERT
        #define	ASSERT(exp)	{}
    #endif
    #define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

    #define	Log				{}
//    #define	SetIceError		false
    #define	EC_OUTOFMEMORY	"Out of memory"

//	#include ".\Ice\IcePreprocessor.h"

    #undef ICECORE_API
    #define ICECORE_API	OPCODE_API

//	#include ".\Ice\IceTypes.h"
//	#include ".\Ice\IceFPU.h"
//	#include ".\Ice\IceMemoryMacros.h"

    namespace IceCore
    {
        #include "IceUtils.h"
        #include "IceContainer.h"
        #include "IcePairs.h"
        #include "IceRevisitedRadix.h"
        #include "IceRandom.h"
    }
    using namespace IceCore;

    #define ICEMATHS_API	OPCODE_API
    namespace IceMaths
    {
        #include "IceAxes.h"
        #include "IcePoint.h"
        #include "IceHPoint.h"
        #include "IceMatrix3x3.h"
        #include "IceMatrix4x4.h"
        #include "IcePlane.h"
        #include "IceRay.h"
        #include "IceIndexedTriangle.h"
        #include "IceTriangle.h"
        #include "IceTrilist.h"
        #include "IceAABB.h"
        #include "IceOBB.h"
        #include "IceBoundingSphere.h"
        #include "IceSegment.h"
        #include "IceLSS.h"
    }
    using namespace IceMaths;

