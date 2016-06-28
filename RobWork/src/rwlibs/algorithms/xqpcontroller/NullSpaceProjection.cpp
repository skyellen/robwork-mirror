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


#include "NullSpaceProjection.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Jacobian.hpp>
#include <rwlibs/algorithms/qpcontroller/QPSolver.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

using namespace rwlibs::algorithms;

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace boost::numeric::ublas;
using namespace rwlibs::algorithms::qpcontroller;

NullSpaceProjection::NullSpaceProjection(Device* device, Frame* controlFrame, const State& state, double dt):
    _state(state),
    _dt(dt),
    _P(identity_matrix<double>(6)),
    _space(BaseFrame)
{
    _device = device;
    _controlFrame = controlFrame;
    _dof = (int)_device->getDOF();

    _qlower = _device->getBounds().first;
    _qupper = _device->getBounds().second;

    _dqlimit = _device->getVelocityLimits();

    _ddqlimit = _device->getAccelerationLimits();

    setThreshold(0.20); //Only do self motion if joint is within the outmost 20% of its value

	_weightJointLimits = 1;
}

NullSpaceProjection::~NullSpaceProjection()
{
}


Q NullSpaceProjection::getGradient(const Q& q) {
    Q g(_dof);
    for (int i = 0; i<_dof; i++) {
        if (q(i) > _thresholdUpper(i))
            g(i) = _weightJointLimits*(q(i) - _thresholdUpper(i))/(_qupper(i) - _qlower(i));
        else if (q(i) < _thresholdLower(i))
            g(i) = _weightJointLimits*(q(i) - _thresholdLower(i))/(_qupper(i) - _qlower(i));
        else
            g(i) = 0;
    }
    return -g;
}


Q NullSpaceProjection::solve(const Q& q, const Q& dqcurrent, const Q& dq1) {
    Q lower(_dof);
    Q upper(_dof);

    calculateVelocityLimits(lower, upper, q, dqcurrent);
    lower -= dq1;
    upper -= dq1;

    _device->setQ(q, _state);

    //Calculate the right projection depending on whether it is in the base frame or the frame of control
    matrix<double> P;
    if (_space == ControlFrame) {
        Rotation3D<> rot = inverse(_device->baseTframe(_controlFrame, _state).R());
        matrix<double> R = zero_matrix<double>(6,6);
        matrix_range<matrix<double> > rot1(R, range(0,3), range(0,3));
        rot1 = rot.m();
        matrix_range<matrix<double> > rot2(R, range(3,6), range(3,6));
        rot2 = rot.m();
        P = prod(_P,R);
    } else {
        P = _P;
    }
    //Get the Jacobian and make the projection
    matrix<double> jac = prod(P, _device->baseJframe(_controlFrame, _state).m());
    //matrix<double> jac = _device->baseJframe(_controlFrame, _state).m();

    matrix<double> jac_inv = LinearAlgebra::pseudoInverse(jac);

    matrix<double> jac_ort = identity_matrix<double>(_dof) - prod(jac_inv, jac);

    matrix<double> cmat(2*jac_ort.size1(), jac_ort.size2());

    matrix_range<matrix<double> > cmat1(cmat, range(0, jac_ort.size1()), range(0, jac_ort.size2()));
    cmat1 = jac_ort;

    matrix_range<matrix<double> > cmat2(cmat, range(jac_ort.size1(), 2*jac_ort.size1()), range(0, jac_ort.size2()));

    cmat2 = -jac_ort;

    vector<double> cvec(2*jac_ort.size1());
    vector_range<vector<double> > cvec1(cvec, range(0, lower.size()));
    cvec1 = lower.m();
    vector_range<vector<double> > cvec2(cvec, range(lower.size(), 2*lower.size()));
    cvec2 = -upper.m();


    matrix<double> jTj = prod(trans(jac_ort), jac_ort);
    vector<double> jTx = prod(trans(jac_ort), getGradient(q).m());

    vector<double> qstart = zero_vector<double>(_dof);

    //std::cout<<"cvec = "<<cvec<<std::endl;

    QPSolver::Status status;
    vector<double> res = QPSolver::inequalitySolve(jTj, -1*jTx, cmat, cvec, qstart, status);
    if (status == QPSolver::SUBOPTIMAL)
        std::cout<<"Returns something suboptimal"<<std::endl;
    if (status == QPSolver::FAILURE)
        std::cout<<"Returns Error"<<std::endl;

    vector<double> nsp = prod(jac_ort, res);

    return Q(nsp);
}


void NullSpaceProjection::calculateVelocityLimits(Q& lower,
                                                  Q& upper,
                                                  const Q& q,
                                                  const Q& dq)
{
    Q joint_pos = q;
    Q joint_vel = dq;

    double accmin, accmax, velmin, velmax, posmin, posmax;
    double x;

    for (int i = 0; i<_dof; i++) {
        //For the acceleration
        accmin = _dt*(-_ddqlimit)[i]+joint_vel(i);
        accmax = _dt*_ddqlimit[i]+joint_vel(i);
        //For the velocity
        velmin = -_dqlimit[i];
        velmax = _dqlimit[i];
        //For the position
        //If v_current<=v_max(X)
        x = _qupper[i]-joint_pos(i);
        if (x<=0) {
            posmax = 0;
            //  std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
        } else {
            //For qmax
            double j_x = Math::round(sqrt(1-8*x/(_dt*_dt*(-_ddqlimit)[i]))/2-1);
            double q_end_x = (x+_dt*_dt*(-_ddqlimit)[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_max_x = q_end_x-j_x*(-_ddqlimit)[i]*_dt;
            double X = x-_dt*q_max_x;
            if (X<=0){
                posmax = 0;
                //          std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
            } else {
                double j_X = Math::round(sqrt(1.-8*X/(_dt*_dt*(-_ddqlimit)[i]))/2.-1);
                double q_end_X = (X+_dt*_dt*(-_ddqlimit)[i]*(j_X*(j_X+1))/2)/(_dt*(j_X+1));
                posmax = q_end_X-j_X*(-_ddqlimit)[i]*_dt;
            }
        }
        x = joint_pos(i)-_qlower[i];
        if (x<=0)    {
            //  std::cout<<"Warning: Set lower pos limit to 0 because"<<x<<"<=0"<<std::endl;
            posmin = 0;
        }else {//For qmin
            double j_x = Math::round(sqrt(1+8*x/(_dt*_dt*_ddqlimit[i]))/2-1);
            double q_end_x = (-x+_dt*_dt*_ddqlimit[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_min_x = q_end_x-j_x*_ddqlimit[i]*_dt;
            double X = x+_dt*q_min_x;
            if (X<=0) {
                posmin = 0;
                //   std::cout<<"Warning: Set lower pos limit to 0"<<x<<"<=0"<<std::endl;
            }else {
                double j_X = Math::round(sqrt(1+8*X/(_dt*_dt*_ddqlimit[i]))/2-1);
                double q_end_X = (-X+_dt*_dt*_ddqlimit[i]*(j_X*(j_X+1))/2)/(_dt*(j_X+1));
                    posmin = q_end_X-j_X*_ddqlimit[i]*_dt;
            }
        }
        upper(i) = std::min(accmax,std::min(velmax, posmax));
        lower(i) = std::max(accmin,std::max(velmin, posmin));

        //Because of numerical uncertainties we need to test whether upper>lower.
        if (upper(i) < lower(i)) {
            lower(i) = upper(i);
        }
    }
}


void NullSpaceProjection::setProjection(const boost::numeric::ublas::matrix<double>& P, ProjectionFrame space) {
    _P = P;
    _space = space;
}

void NullSpaceProjection::setThreshold(double threshold) {
    _thresholdLower = _qlower + threshold*(_qupper-_qlower);
    _thresholdUpper = _qupper - threshold*(_qupper-_qlower);
}

void NullSpaceProjection::setJointLimitsWeight(double w) {
    _weightJointLimits = w;
}
