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

#include "JacobianIKSolver.hpp"

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
#include <rw/trajectory/LinearInterpolator.hpp>

using namespace boost;
using namespace boost::numeric;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;
using namespace rw::trajectory;

JacobianIKSolver::JacobianIKSolver(Device::Ptr device, Frame *foi, const State& state):
    _device(device),
    _interpolationStep(0.21),
    _fkrange( device->getBase(), foi, state),
    _devJac( device->baseJCframe(foi,state) ),
    _useJointClamping(false)
{
    setMaxIterations(15);
}

JacobianIKSolver::JacobianIKSolver(Device::Ptr device, const State& state):
    _device(device),
    _interpolationStep(0.21),
    _fkrange( device->getBase(), device->getEnd(), state),
    _devJac( device->baseJCend(state) ),
    _useJointClamping(false)
{
    setMaxIterations(15);
}

std::vector<Q> JacobianIKSolver::solve(const Transform3D<>& bTed,
                                   const State& initial_state) const
{
    int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    // if the distance between current and end configuration is
    // too large then split it up in smaller steps
    const Transform3D<>& bTeInit = _fkrange.get(state);

    if( _useInterpolation ){
        LinearInterpolator<Transform3D<> > interpolator(bTeInit, bTed, _interpolationStep);
        for(double t=_interpolationStep; t<interpolator.duration(); t+=_interpolationStep){
            Transform3D<> bTed_via = interpolator.x(t);
            /* bool found = */ solveLocal(bTed_via, maxError*1000, state, 5);
        }
    }

    // now we perform yet another newton search with higher precision to determine
    // the end result
    if (solveLocal(bTed, maxError, state, maxIterations ) )
    {
        std::vector<Q> result;
        Q q = _device->getQ(state);
        if (!_checkJointLimits || Models::inBounds(q, *_device))
            result.push_back(q); 
        return result;
    }

    return std::vector<Q>();
}

bool JacobianIKSolver::solveLocal(const Transform3D<> &bTed,
                              double maxError,
                              State &state,
                              int maxIter) const
{
    Q q = _device->getQ(state);
    const int maxIterations = maxIter;
    Device::QBox bounds = _device->getBounds();
    LinearAlgebra::Matrix<double>::type Jp;


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

        const ublas::vector<double> dS = b_eXed.m();
        Jacobian J = _devJac->get( state );
        switch(_solverType){
        case(Transpose):
        {
            Jp = trans( J.m() );

            ublas::vector<double> dTheta = prod( Jp , dS);
            ublas::vector<double> dT = prod( J.m() , dTheta);
            double alpha = inner_prod(dS,dT)/ Math::sqr( norm_2(dT) );
            RW_ASSERT(alpha>0.0);

            double maxChange = norm_inf( dTheta );
            double beta = 0.8/maxChange;
            dTheta *= std::min(alpha, beta);
            Q dq(dTheta);
            q += dq;
        }
        break;
        case(SVD):{
            Jp = LinearAlgebra::pseudoInverse(J.m());
            Q dq ( prod( Jp , dS ) );
            double dq_len = dq.normInf();
            if( dq_len > 0.8 )
                dq *= 0.8/dq_len;
            q += dq;
        }
        break;
        case(SDLS):{
            // TODO: not implemented yet for now we just use DLS
        }
        case(DLS):{
            //std::cout << "Error: " << cnt << " : "<< error << std::endl;
            double lambda = 0.4; // dampening factor, for now a fixed value
            //std::cout << "1";
            ublas::matrix<double> U = prod( J.m(), trans(J.m()) ); // U = J * (J^T)
            //std::cout << "2";
            ublas::identity_matrix<double> I ( U.size2() );
            //std::cout << "3";
            U = U + I*lambda;
            //std::cout << "4";
            ublas::matrix<double> Uinv(U.size1(),U.size2());
            //std::cout << "5";
            LinearAlgebra::invertMatrix(U, Uinv);
            //std::cout << "6";
            ublas::vector<double> dT = prod(Uinv, dS);
            //std::cout << "7";
            // Use these two lines for the traditional DLS method
            ublas::vector<double> dTheta = prod( trans(J.m()), dT );
            //std::cout << "8";
            // Scale back to not exceed maximum angle changes
            double maxChange = norm_inf( dTheta );
            if ( maxChange>45.0*Deg2Rad) {
                dTheta *= (45.0*Deg2Rad)/maxChange;
            }
            Q dq(dTheta);
            q += dq;
        }
        break;

        }

        if(_useJointClamping)
            q = Math::clampQ(q, bounds.first, bounds.second);
        _device->setQ(q, state);
    }
    return false;
}

