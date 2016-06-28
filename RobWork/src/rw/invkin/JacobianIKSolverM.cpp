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

#include "JacobianIKSolverM.hpp"

#include <rw/kinematics/FKRange.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Math.hpp>

#include <rw/models/TreeDevice.hpp>
#include <rw/models/Device.hpp>

#include <boost/shared_ptr.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#define MAX_ANGLE_JTRANSPOSE 30*Deg2Rad


using namespace boost;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::invkin;
using namespace rw::trajectory;

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

JacobianIKSolverM::JacobianIKSolverM(const TreeDevice* device,
                                     const State& state):
    IterativeMultiIK(device->getEnds().size()),
    _device(device),
    _foi(device->getEnds()),
    _fkranges( createFKRanges(device->getBase(),_foi,state) ),
    _interpolationStep(0.2),
    _returnBestFit(false),
    _useJointClamping(true), _useInterpolation(true),
    _solverType(JacobianIKSolverM::SVD)
{
    setMaxIterations(40);
    _jacCalc = device->baseJCframes( _foi, state );
}

JacobianIKSolverM::JacobianIKSolverM(const JointDevice* device,
                                     const std::vector<Frame*>& foi,
                                     const State& state):
     IterativeMultiIK(foi.size()),
     _device(device),
     _foi(foi),
     _fkranges(createFKRanges(device->getBase(),_foi,state)),
     _interpolationStep(0.2),
     _returnBestFit(false),
     _useJointClamping(true), _useInterpolation(true),
     _solverType(JacobianIKSolverM::SVD)
 {
     setMaxIterations(40);
     _jacCalc = device->baseJCframes( foi, state );
 }

bool JacobianIKSolverM::solveLocal(
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
	Eigen::VectorXd b_eXed_vec(_fkranges.size()*6);
	Eigen::MatrixXd Jp;
    Device::QBox bounds = _device->getBounds();

    double lastError = 10000;
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
		double error = b_eXed_vec.norm();
        //std::cout << "Error: " << cnt << " : "<< error << std::endl;
        if(!untilSmallChange){
            if( error  < maxError[0]/*belowMaxError*/ ){
                //std::cout << "below MAX Error: " << error << " "<<  b_eXed_vec << std::endl;
                return true;
            }
            /*if(error-lastError>0){
                return false;
            }*/
        } else {
            if( std::fabs(error-lastError)<maxError[0] /*|| error-lastError>0*/)
                return true;

        }
        lastError = error;

        const Eigen::VectorXd& dS = b_eXed_vec;
        Jacobian J = _jacCalc->get( state );
        switch(_solverType){
        case(Transpose):
        {
			Jp = J.e().transpose();

            Eigen::VectorXd dTheta = Jp*dS;//prod( Jp , dS);
            Eigen::VectorXd dT = J.e()*dTheta;//prod( J.m() , dTheta);
            //double alpha = inner_prod(b_eXed_vec,dT)/ Math::sqr( norm_2(dT) );
			double alpha = b_eXed_vec.dot(dT);
            RW_ASSERT(alpha>0.0);

			double maxChange = dTheta.lpNorm<Eigen::Infinity>();
            double beta = 0.8/maxChange;
            dTheta *= std::min(alpha, beta);
            Q dq(dTheta);
            q += dq;
        }
        break;
        case(SVD):{
            //std::cout << " size: "  << J.m().size1() <<  " < "<< J.m().size2() << std::endl;
            /*ublas::identity_matrix<double> I ( J.m().size1() );
            ublas::matrix<double> W = I;
            //std::cout << "3";
            for(size_t i=0;i<J.m().size1()/6;i++){
                int off = i*6;
                W(off+3,off+3) = 4;
                W(off+4,off+4) = 4;
                W(off+5,off+5) = 4;
            }*/
            //std::cout << " size: "  << W.size1() <<  " < "<< W.size2() << std::endl;
            //Jp = LinearAlgebra::pseudoInverse( prod(W, J.m()) );
            Jp = LinearAlgebra::pseudoInverse(  J.e() );
            //Q dq ( prod( Jp , b_eXed_vec) );
			Q dq ( Jp*b_eXed_vec);
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
            //ublas::matrix<double> U = prod( J.m(), trans(J.m()) ); // U = J * (J^T)
			Eigen::MatrixXd U = J.e() * J.e().transpose();
            //ublas::identity_matrix<double> I ( U.size2() );
			U = U + lambda*Eigen::MatrixXd::Identity(U.rows(), U.cols());
            Eigen::MatrixXd Uinv = U.inverse();
            //LinearAlgebra::invertMatrix(U, Uinv);
            Eigen::VectorXd dT = Uinv*dS;
            // Use these two lines for the traditional DLS method
			Eigen::VectorXd dTheta = J.e().transpose()*dT;
            // Scale back to not exceed maximum angle changes
			double maxChange = dTheta.lpNorm<Eigen::Infinity>();
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


std::vector<Q> JacobianIKSolverM::solve(
    const std::vector<Transform3D<> >& bTeds,
    const State& initial_state) const

{
    int maxIterations = getMaxIterations();
    std::vector<double> maxError = getMaxError();
    State state = initial_state;

    const int N = (int)bTeds.size();

    if( _useInterpolation ){
        // for each end effector calculate the distance
        std::vector<LinearInterpolator<Transform3D<> > > interpolators;
        for(int i=0; i<N; i++){
            Transform3D<> bTeInit = _fkranges[i]->get(state);
            interpolators.push_back( LinearInterpolator<Transform3D<> >(bTeInit, bTeds[i], _interpolationStep) );
        }

        std::vector<Transform3D<> > bTeds_via(N);
        for(double t=_interpolationStep; t<interpolators[0].duration(); t+=_interpolationStep){
            // calculate the bTes
            for(int i=0;i<N;i++)
                bTeds_via[i] = interpolators[i].x(t);
            // now perform newton iteration to the generated via point
            // we relax the error requirement a bit to alow for faster search
            solveLocal(bTeds_via, maxError, state, 5, true);
        }
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
