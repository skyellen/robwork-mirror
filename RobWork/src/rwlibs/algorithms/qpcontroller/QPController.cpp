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


#include "QPController.hpp"
#include "QPSolver.hpp"

#include <rw/models/Device.hpp>
#include <rw/common/macros.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <cmath>

using namespace boost::numeric::ublas;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

using namespace rwlibs::algorithms;
using namespace rwlibs::algorithms::qpcontroller;

QPController::QPController(double h, const State& state, Device* device):
    _h(h),
    _state(state)
 {
     RW_ASSERT(device != NULL);
     _device = device;
     _n = device->getDOF();

     _qmin = _device->getBounds().first.m();
     _qmax = _device->getBounds().second.m();

     _vmax = _device->getVelocityLimits().m();
     _vmin = _vmax*(-1.0);

     _amax = _device->getAccelerationLimits().m();
     _amin = _amax*(-1.0);

     _statusArray = new char[256];
     _statusArray[0] = 0;
     _lowerLimitType = new char[_n];
     _upperLimitType = new char[_n];

     _accLimit = new int[_n];
     _velLimit = new int[_n];
     _posLimit = new int[_n];

}

QPController::~QPController() {
    delete[] _statusArray;
    delete[] _lowerLimitType;
    delete[] _upperLimitType;
    delete[] _accLimit;
    delete[] _velLimit;
    delete[] _posLimit;
}

Q QPController::solve(const Q& q, const Q& dq, const VelocityScrew6D<>& tcp_screw)
{
    if (q.size() != _n || dq.size() != _n) {
        RW_THROW("Length of input configuration does not match the Device");
    }

    VectorBase tcp_vel(6);
    Vector3D<> linvel = tcp_screw.linear();
    EAA<> angvel = tcp_screw.angular();

    tcp_vel(0) = linvel(0);
    tcp_vel(1) = linvel(1);
    tcp_vel(2) = linvel(2);
    tcp_vel(3) = angvel.angle()*angvel.axis()(0);
    tcp_vel(4) = angvel.angle()*angvel.axis()(1);
    tcp_vel(5) = angvel.angle()*angvel.axis()(2);

    /*    for (int i = 0; i<3; i++) {
        tcp_vel(i) = linvel(i);
        tcp_vel(i+3) = angvel.angle()*angvel.axis()(i);
        }*/
    _device->setQ(q, _state);

    matrix<double> jac = _device->baseJend(_state).m();
    //trim jacobian to remove rool rotation
    //    matrix_range<matrix<double> > jac(jac6, range(0,6), range(0, jac6.size2()));

    matrix<double> A = prod(trans(jac),jac);
    VectorBase b = prod(trans(jac),tcp_vel);

    VectorBase lower(_n);
    VectorBase upper(_n);
    calculateVelocityLimits(lower, upper, q.m(), dq.m());
    VectorBase sol1 = inequalitySolve(A, b, lower, upper);

    return Q(sol1);
}

void QPController::calculateVelocityLimits(
    VectorBase& lower,
    VectorBase& upper,
    const VectorBase& q,
    const VectorBase& dq)
{
    VectorBase joint_pos = q;
    VectorBase joint_vel = dq;
    double accmin, accmax, velmin, velmax, posmin, posmax;
    double x;
    for (size_t i = 0; i<_n; i++) {
        //For the acceleration
        accmin = _h*_amin[i]+joint_vel(i);
        accmax = _h*_amax[i]+joint_vel(i);
        //For the velocity
        velmin = _vmin[i];
        velmax = _vmax[i];
        //For the position
        //If v_current<=v_max(X)
        x = _qmax[i]-joint_pos(i);
        if (x<=0) {
            posmax = 0;
            //  std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
        } else {
            //For qmax
            double j_x = Math::round(sqrt(1-8*x/(_h*_h*_amin[i]))/2-1);
            double q_end_x = (x+_h*_h*_amin[i]*(j_x*(j_x+1))/2)/(_h*(j_x+1));
            double q_max_x = q_end_x-j_x*_amin[i]*_h;
            double X = x-_h*q_max_x;
            if (X<=0){
                posmax = 0;
                //          std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
            } else {
                double j_X = Math::round(sqrt(1.-8*X/(_h*_h*_amin[i]))/2.-1);
                double q_end_X = (X+_h*_h*_amin[i]*(j_X*(j_X+1))/2)/(_h*(j_X+1));
                posmax = q_end_X-j_X*_amin[i]*_h;
            }
        }
        x = joint_pos(i)-_qmin[i];
        if (x<=0)    {
            //  std::cout<<"Warning: Set lower pos limit to 0 because"<<x<<"<=0"<<std::endl;
            posmin = 0;
        }else {//For qmin
            double j_x = Math::round(sqrt(1+8*x/(_h*_h*_amax[i]))/2-1);
            double q_end_x = (-x+_h*_h*_amax[i]*(j_x*(j_x+1))/2)/(_h*(j_x+1));
            double q_min_x = q_end_x-j_x*_amax[i]*_h;
            double X = x+_h*q_min_x;
            if (X<=0) {
                posmin = 0;
                //   std::cout<<"Warning: Set lower pos limit to 0"<<x<<"<=0"<<std::endl;
            }else {
                double j_X = Math::round(sqrt(1+8*X/(_h*_h*_amax[i]))/2-1);
                double q_end_X = (-X+_h*_h*_amax[i]*(j_X*(j_X+1))/2)/(_h*(j_X+1));
                    posmin = q_end_X-j_X*_amax[i]*_h;
            }
        }
        upper(i) = std::min(accmax,std::min(velmax, posmax));
        lower(i) = std::max(accmin,std::max(velmin, posmin));

        if (upper(i) == accmax)
            _upperLimitType[i] = 'a';
        else if (upper(i) == velmax)
            _upperLimitType[i] = 'v';
        else
            _upperLimitType[i] = 'q';

        if (lower(i) == accmin)
            _lowerLimitType[i] = 'a';
        else if (lower(i) == velmin)
            _lowerLimitType[i] = 'v';
        else
            _lowerLimitType[i] = 'q';

        //Because of numerical uncertainties we need to test whether upper>lower.
        if (upper(i) < lower(i)) {
            lower(i) = upper(i);
            //  std::cout<<"Warning: Upper set to be lower "<<i<<std::endl;
        }
        //if (upper(i) == lower(i))
        //std::cout<<"Warning: Upper and Lower is equal "<<std::endl;
    }
}

QPController::VectorBase QPController::inequalitySolve(
    const matrix<double>& G,
    const VectorBase& b,
    const VectorBase& lower,
    const VectorBase& upper)
{
    matrix<double> cmat = zero_matrix<double>(2*lower.size(), lower.size());
    VectorBase limits(2*lower.size());
    for (size_t i = 0; i<lower.size(); i++) {
        cmat(2*i,i) = 1;
        cmat(2*i+1,i) = -1;
        limits(2*i) = lower(i);
        limits(2*i+1) = -upper(i);
    }

    QPSolver::Status status;
    VectorBase qstart = (lower + upper) / 2.0;
    VectorBase res = QPSolver::inequalitySolve(
        G, b * (-1), cmat, limits, qstart, status);

    if (status == QPSolver::FAILURE) {
        RW_WARN("Error QPSolver could not solve with a valid solution");
        return Q::zero(lower.size()).m();
    }
    /*    for (size_t i = 0; i<res.size(); i++) {
        if (res(i) > upper(i)) {
            res(i) = upper(i);
        } else if (res(i) < lower(i)) {
            res(i) = lower(i);
        }
        }*/

    return res;
}
