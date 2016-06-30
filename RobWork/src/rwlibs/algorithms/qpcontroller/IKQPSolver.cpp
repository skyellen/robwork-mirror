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


#include "IKQPSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/models/SerialDevice.hpp>

using namespace rwlibs::algorithms;

using namespace boost;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;

const double DT = 0.5;

IKQPSolver::IKQPSolver(SerialDevice* device, const State& state):
    _qpcontroller(DT, state, device),
    _device(device),
    _state(state),
    _maxQuatStep(0.8)
{}



bool IKQPSolver::performLocalSearch(const SerialDevice *device,
                        			const Transform3D<> &bTed,
                        			double maxError,
                        			State &state,
                        			unsigned int maxIter) const {
	QPController* qpcontroller = const_cast<QPController*>(&_qpcontroller);
    int maxIterations = maxIter;
    Q dq(Q::zero(device->getDOF()));
    while (maxIterations--) {

        const Transform3D<>& Tcurrent = _device->baseTend(state);

        VelocityScrew6D<> vs(inverse(Tcurrent)*bTed);
        VelocityScrew6D<> diff = Tcurrent.R()*vs;
        //std::cout<<"diff = "<<norm_inf(diff)<<std::endl;
        if (normInf(diff) <= maxError) {
        	return true;
        }

        if (norm2(diff) > 1)
            diff *= 1/norm2(diff);

        dq = qpcontroller->solve(device->getQ(state), dq, diff);
        const Q q = device->getQ(state) + DT*dq;

        device->setQ(q, state);
    }
    return true;
}








std::vector<Q> IKQPSolver::solve(
    const Transform3D<>& bTed,
    const State& initial_state) const

{
    unsigned int maxIterations = getMaxIterations();
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
        Quaternion<> qNext(0,0,0,1);
        for (int i = 0; i<4; i++)
        	qNext(i) = q1(i)+qDist(i)*step/steps;
        qNext.normalize();
        Vector3D<> pNext = bTeInit.P()+posDist*step/steps;
        Transform3D<> bTedLocal(pNext, qNext);
        // we allow a relative large error since its only via points
        performLocalSearch(_device, bTedLocal, maxError*1000, state, maxIterations );
    }
    // now we perform yet another newton search with higher precision to determine
    // the end result
    performLocalSearch(_device, bTed, maxError, state, maxIterations );
    std::vector<Q> result;
    result.push_back(_device->getQ(state));
    return result;
}
