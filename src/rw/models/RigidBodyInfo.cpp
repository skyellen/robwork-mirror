#include "RigidBodyInfo.hpp"

using namespace rw::math;
using namespace rw::models;

RigidBodyInfo::RigidBodyInfo(double mass,
                             const InertiaMatrix<>& Ibody):
    _mass(mass),
    _Ibody(Ibody)
{
}

RigidBodyInfo::~RigidBodyInfo()
{
}
