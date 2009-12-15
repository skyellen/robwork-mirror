/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SimpleSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/common/Property.hpp>

#include <rw/models/Models.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>

#include <boost/shared_ptr.hpp>

using namespace boost;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;

SimpleSolver::SimpleSolver(Device* device, Frame *foi, const State& state):
    _device(device),
    _maxQuatStep(0.4),
    _fkrange( device->getBase(), foi, state),
    _devJac( device->baseJCframe(foi,state) ),
    _checkForLimits(false)
{
    setMaxIterations(15);
}

SimpleSolver::SimpleSolver(Device* device, const State& state):
    _device(device),
    _maxQuatStep(0.4),
    _fkrange( device->getBase(), device->getEnd(), state),
    _devJac( device->baseJCend(state) )
{
    setMaxIterations(15);
}

void SimpleSolver::setMaxLocalStep(double quatlength, double poslength)
{
    _maxQuatStep = quatlength;
}

void SimpleSolver::setCheckJointLimits(bool check) {
    _checkForLimits = check;
}


std::vector<Q> SimpleSolver::solve(const Transform3D<>& bTed,
                                   const State& initial_state) const
{
    int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    // if the distance between current and end configuration is
    // too large then split it up in smaller steps
    const Transform3D<>& bTeInit = _fkrange.get(state);
    Quaternion<> q1( bTeInit.R() );
    Quaternion<> q2( bTed.R() );
    Quaternion<> qDist = q2-q1;
    double length = qDist.getLength();
    int steps = (int)ceil( length/_maxQuatStep );
    Vector3D<> posDist = bTed.P()-bTeInit.P();

    //std::cout << "Steps: " << steps << std::endl;
    //std::cout << "INIT: "<< q1 << " "  << bTeInit.P() << std::endl;
    //std::cout << "GOAL: "<< q2 << " "  << bTed.P() << std::endl;
    // now perform newton iterations to each generated via point
    for(int step=1; step < steps; step++){
        // calculate
        //std::cout << step;
        double nStep = ((double)step) / (double)steps;
        Quaternion<> qNext = qDist;
        qNext *= nStep;
        //qNext.normalize();
        qNext = q1 + qNext;
        qNext.normalize();
        Vector3D<> pNext = bTeInit.P() + posDist*nStep;
        Transform3D<> bTedLocal(pNext,qNext);

        // we allow a relative large error since its only via points
        //std::cout << qNext << " " << pNext << std::endl;
        /* bool found = */ solveLocal(bTedLocal, maxError*1000, state, 5);
    }

    // now we perform yet another newton search with higher precision to determine
    // the end result
    if (solveLocal(bTed, maxError, state, maxIterations ) )
    {
        std::vector<Q> result;
        Q q = _device->getQ(state);
        if (!_checkForLimits || Models::inBounds(q, *_device))
            result.push_back(q); 
        return result;
    }

    return std::vector<Q>();
}

bool SimpleSolver::solveLocal(const Transform3D<> &bTed,
                              double maxError,
                              State &state,
                              int maxIter) const
{
    Q q = _device->getQ(state);
    const int maxIterations = maxIter;


    for (int cnt = 0; cnt < maxIterations; ++cnt) {
        const Transform3D<>& bTe = _fkrange.get(state);
        const Transform3D<>& eTed = inverse(bTe) * bTed;

        const EAA<> e_eOed(eTed(2,1), eTed(0,2), eTed(1,0));
        const Vector3D<>& e_eVed = eTed.P();
        const VelocityScrew6D<> e_eXed(e_eVed, e_eOed);
        const VelocityScrew6D<>& b_eXed = bTe.R() * e_eXed;

        if (norm_inf(b_eXed) <= maxError) {
            return true;
        }

        const Jacobian& J = _devJac->get(state);
        const Jacobian& Jp = Jacobian(LinearAlgebra::pseudoInverse(J.m()));

        Q dq = Jp * b_eXed;
        double dq_len = dq.normInf();
        if( dq_len > 0.8 )
            dq *= 0.8/dq_len;

        q += dq;
        _device->setQ(q, state);
    }
    return false;
}

