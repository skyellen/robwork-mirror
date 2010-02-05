#include "SpherePoseSampler.hpp"

using namespace rw::kinematics;
using namespace rw::math;

SpherePoseSampler::SpherePoseSampler(MovableFrame* mframe,
                                     const Vector3D<>& wPc, State& initState) :
    _mframe(mframe),
    _wPc(wPc),
    _initState(initState)
{
}

SpherePoseSampler::~SpherePoseSampler()
{
}

bool SpherePoseSampler::sample(State& state)
{
    return true;
}

