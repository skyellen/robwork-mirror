/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#include "IKQPSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/common/Property.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Quaternion.hpp>

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
    Q dq(Q::ZeroBase(device->getDOF()));
    while (maxIterations--) {
 
        const Transform3D<>& Tcurrent = _device->baseTend(state);

        VelocityScrew6D<> vs(inverse(Tcurrent)*bTed);        
        VelocityScrew6D<> diff = Tcurrent.R()*vs;
        //std::cout<<"diff = "<<norm_inf(diff)<<std::endl;
        if (norm_inf(diff) <= maxError) {
        	return true;
        }        

        if (norm_2(diff) > 1)
            diff *= 1/norm_2(diff);
       
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
