
#include "BVTreeCollider.hpp"

#include <rw/math/Transform3D.hpp>

#include "OBBCollider.hpp"

using namespace rw::proximity;
using namespace rw::math;
using namespace rw::geometry;


BVTreeCollider<BinaryOBBPtrTreeD>* BVTreeColliderFactory::makeOBBPtrTreeBDFSColliderD(){
    return makeBalancedDFSColliderOBB<BinaryOBBPtrTreeD>();
}

BVTreeCollider<BinaryOBBPtrTreeF>* BVTreeColliderFactory::makeOBBPtrTreeBDFSColliderF(){
    return makeBalancedDFSColliderOBB<BinaryOBBPtrTreeF>();
}

