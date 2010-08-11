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

#include "SimpleMultiSolver.hpp"

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Quaternion.hpp>

#include <rw/common/Property.hpp>

#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/JacobianCalculator.hpp>
#include <rw/models/WorkCell.hpp>

#include <boost/shared_ptr.hpp>

using namespace boost;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;

using namespace boost::numeric;

namespace {

    std::vector<boost::shared_ptr<FKRange> > createFKRanges(const Frame *base,
                                                            const std::vector<Frame*>& foi,
                                                            const State& state)
    {
        std::vector<boost::shared_ptr<FKRange> > fkranges;
        for(size_t i = 0; i<foi.size(); i++){
            boost::shared_ptr<FKRange> range(new FKRange(base , foi[i], state) );
            fkranges.push_back( range );
        }
        return fkranges;
    }


}

SimpleMultiSolver::SimpleMultiSolver(const TreeDevice* device,
                                     const State& state):
    IterativeMultiIK(device->getEnds().size()),
    _device(device),
    _foi(device->getEnds()),
    _fkranges( createFKRanges(device->getBase(),_foi,state) ),
    _maxQuatStep(0.4),
    _returnBestFit(false)
{
    setMaxIterations(40);
    _jacCalc = device->baseJCframes( _foi, state );
}

SimpleMultiSolver::SimpleMultiSolver(const JointDevice* device,
                                     const std::vector<Frame*>& foi,
                                     const State& state):
     IterativeMultiIK(foi.size()),
     _device(device),
     _foi(foi),
     _fkranges(createFKRanges(device->getBase(),_foi,state)),
     _maxQuatStep(0.4),
     _returnBestFit(false)
 {
     setMaxIterations(40);
     _jacCalc = device->baseJCframes( foi, state );
 }

void SimpleMultiSolver::setMaxLocalStep(double quatlength, double poslength){
    _maxQuatStep = quatlength;
}

bool SimpleMultiSolver::solveLocal(
    const std::vector<Transform3D<> > &bTed,
    std::vector<double>& maxError,
    State &state,
    int maxIter,
    bool untilSmallChange) const
{
    //std::cout << "Perform local search!" << std::endl;

    //std::cout << bTed.size() << "==" << fkranges.size() << std::endl;
    RW_ASSERT(bTed.size()==_fkranges.size());
    //std::cout << "Create vector " << std::endl;
    boost::numeric::ublas::vector<double> b_eXed_vec(_fkranges.size()*6);
    double lastError = 0;
    Q q = _device->getQ(state);
    const int maxIterations = maxIter;
    for (int cnt = 0; cnt < maxIterations; ++cnt) {
        //std::cout << "Iteration: " << maxIterations << std::endl;
        //bool belowMaxError = true;
        for(size_t row=0;row<_fkranges.size(); row++){
            Transform3D<> bTe = _fkranges[row]->get(state);
            Transform3D<> eTed = inverse(bTe) * bTed[row];

            EAA<> e_eOed(eTed(2,1), eTed(0,2), eTed(1,0));
            Vector3D<> e_eVed = eTed.P();
            VelocityScrew6D<> e_eXed(e_eVed, e_eOed);
            VelocityScrew6D<> b_eXed = bTe.R() * e_eXed;

            // copy result to b_eXed_vector
            b_eXed_vec(row*6+0) = b_eXed(0);
            b_eXed_vec(row*6+1) = b_eXed(1);
            b_eXed_vec(row*6+2) = b_eXed(2);
            b_eXed_vec(row*6+3) = b_eXed(3);
            b_eXed_vec(row*6+4) = b_eXed(4);
            b_eXed_vec(row*6+5) = b_eXed(5);

            //if ( norm_inf(b_eXed) > maxError[row]*errorScale ) {
                //belowMaxError = false;
            //}
            //std::cout << b_eXed << std::endl;
        }
        double error = norm_2(b_eXed_vec);
        // std::cout << "Error: " << error << std::endl;
        if(!untilSmallChange){
            if( error  < maxError[0]/*belowMaxError*/ ){
                //std::cout << "below MAX Error: " << error << " "<<  b_eXed_vec << std::endl;
                return true;
            }
        } else {
            if( std::fabs(error-lastError)<maxError[0])
                return true;
            lastError = error;
        }

        Jacobian J = _jacCalc->get( state );
        const LinearAlgebra::Matrix<double>::type& Jp = LinearAlgebra::pseudoInverse(J.m());

        Q dq ( prod( Jp , b_eXed_vec) );

        double dq_len = dq.normInf();
        if( dq_len > 0.8 )
            dq *= 0.8/dq_len;

        q += dq;

        _device->setQ(q, state);
    }
    return false;
}


std::vector<Q> SimpleMultiSolver::solve(
    const std::vector<Transform3D<> >& bTeds,
    const State& initial_state) const

{
    int maxIterations = getMaxIterations();
    std::vector<double> maxError = getMaxError();
    std::vector<double> maxTmpError = maxError;
    State state = initial_state;

    const int N = bTeds.size();

    // for each end effector calculate the distance
    double length = 0;
    std::vector<Transform3D<> > bTes(N);
    std::vector<Vector3D<> > pDists(N);
    std::vector<Quaternion<> > q1s(N);
    std::vector<Quaternion<> > q2s(N);
    std::vector<Quaternion<> > qDists(N);
    for(int i=0; i<N; i++){
        bTes[i] = _fkranges[i]->get(state);
        pDists[i] = bTeds[i].P() - bTes[i].P();
        q1s[i] = Quaternion<>( bTes[i].R() );
        q2s[i] = Quaternion<>( bTeds[i].R() );
        qDists[i] = q2s[i]- q1s[i];
        maxTmpError[i] *= 1000;
        double len = qDists[i].getLength();
        if(len>length)
            length = len;
    }
    // now that we have the max length we can calculate the nr of steps

    const int steps = (int)ceil( length/_maxQuatStep );
    //std::cout << "Nr of Steps: " << steps << std::endl;
    // create a vector of the via targets
    std::vector<Transform3D<> > bTedVia(N);
    for(int step=1; step < steps; step++){
        // generate the via point target list from the interpolation
        double nStep = ((double)step) / (double)steps;
        for(int i=0; i<N; i++){
            //Quaternion<> qNext = q1s[i].slerp(q2s[i], nStep);
            Quaternion<> qNext = qDists[i];
            qNext *= nStep;
            qNext = q1s[i] + qNext;
            qNext.normalize();
            Vector3D<> pNext = bTes[i].P() + pDists[i]*nStep;
            bTedVia[i] = Transform3D<>(pNext,qNext);
        }
        // now perform newton iteration to the generated via point
        // we relax the error requirement a bit to alow for faster search
        solveLocal(bTedVia, maxTmpError, state, 5, true);
    }
    // now we perform yet another newton search with higher precision to determine
    // the end result
    std::vector<Q> result;
    if( solveLocal(bTeds, maxError, state, maxIterations, false) ){
        //std::cout << "result found!" << std::endl;
        result.push_back(_device->getQ(state));
    } else if( _returnBestFit ){
        result.push_back(_device->getQ(state));
    }
    return result;
}
