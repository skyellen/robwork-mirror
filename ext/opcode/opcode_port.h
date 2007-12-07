#ifndef _OPCODE_PORT_H_
#define _OPCODE_PORT_H_

#undef WIN32

#define OPCODE_API

// math stuff
//#include "OPC_Point.h"
//#include "OPC_Matrix3x3.h"

#include "OPC_IceHook.h"

typedef int					BOOL;

namespace Opcode
{
    // Bulk-of-the-work

    #include "OPC_Settings.h"
    #include "OPC_Common.h"
    #include "OPC_MeshInterface.h"

    // Builders
    #include "OPC_TreeBuilders.h"
    // Trees
    #include "OPC_AABBTree.h"
    #include "OPC_OptimizedTree.h"
    // Models
    #include "OPC_BaseModel.h"
    #include "OPC_Model.h"
    #include "OPC_HybridModel.h"
    // Colliders
    #include "OPC_Collider.h"
    #include "OPC_VolumeCollider.h"
    #include "OPC_TreeCollider.h"
    #include "OPC_RayCollider.h"
    #include "OPC_SphereCollider.h"
    #include "OPC_OBBCollider.h"
    #include "OPC_AABBCollider.h"
    #include "OPC_LSSCollider.h"
    #include "OPC_PlanesCollider.h"
    // Usages
    #include "OPC_Picking.h"
    // Sweep-and-prune
    #include "OPC_BoxPruning.h"
    #include "OPC_SweepAndPrune.h"

    FUNCTION OPCODE_API bool InitOpcode();
    FUNCTION OPCODE_API bool CloseOpcode();
}

#endif //_OPCODE_PORT_H_

