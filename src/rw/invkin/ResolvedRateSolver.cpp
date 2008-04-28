/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/
#include "ResolvedRateSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Jacobian.hpp>

#include <rw/common/Property.hpp>

#include <rw/kinematics/State.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::invkin;

namespace
{
    bool performLocalSearch(
        const Device& device,
        const Transform3D<> &bTed,
        double maxError,
        State &state,
        int maxIter)
    {
        int maxIterations = maxIter;

        for (int cnt = 0; cnt < maxIterations; ++cnt) {
            const Transform3D<>& bTe = device.baseTend(state);

            const Transform3D<>& eTed = inverse(bTe) * bTed;
            const VelocityScrew6D<> e_eXed(eTed);
            const VelocityScrew6D<>& b_eXed = bTe.R() * e_eXed;

            if (norm_inf(b_eXed) <= maxError) {
                return true;
            }

            const Jacobian& J = device.baseJend(state);
            const Jacobian& Jp = Jacobian(LinearAlgebra::pseudoInverse(J.m()));

            Q dq = Jp * b_eXed;

            double dq_len = dq.normInf();

            if (dq_len > 0.8)
                dq *= 0.8 / dq_len;

            const Q& q = device.getQ(state) + dq;

            device.setQ(q, state);
        }

        return false;
    }
}

ResolvedRateSolver::ResolvedRateSolver(const Device* device) :
    _device(device),_maxQuatStep(0.4)
{
    // If Newtons method has not terminated within a few iterations, it is in
    // practice better to restart the method from a new seed:
    setMaxIterations(15);
}

void ResolvedRateSolver::setMaxLocalStep(double quatlength, double poslength){
    _maxQuatStep = quatlength;
}

std::vector<Q> ResolvedRateSolver::solve(
    const Transform3D<>& bTed,
    const State& initial_state) const
{
    int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    // if the distance between current and end configuration is
    // too large then split it up in smaller steps
    const Transform3D<>& bTeInit = _device->baseTend(state);
    Quaternion<> q1( bTeInit.R() );
    Quaternion<> q2( bTed.R() );
    Quaternion<> qDist = q2-q1;
    double length = qDist.getLength();
    int steps = (int)ceil( length/_maxQuatStep );
    Vector3D<> posDist = bTed.P()-bTeInit.P();

    // now perform newton iterations to each generated via point
    for(int step=1; step < steps; step++){
        // calculate
        //std::cout << step;
        double nStep = ((double)step) / (double)steps;
        Quaternion<> qNext = qDist;
        qNext *= nStep;

        qNext = q1 + qNext;
        qNext.normalize();
        Vector3D<> pNext = bTeInit.P() + posDist*nStep;
        Transform3D<> bTedLocal(pNext,qNext);

        // we allow a relative large error since its only via points
        bool found = performLocalSearch(*_device, bTedLocal, maxError*1000, state, maxIterations );
        if (!found)
            return std::vector<Q>();
    }

    // now we perform yet another newton search with higher precision to determine
    // the end result
    if (performLocalSearch(*_device, bTed, maxError, state, maxIterations)) {
        std::vector<Q> result;
        result.push_back(_device->getQ(state));
        return result;
    }

    return std::vector<Q>();
}
