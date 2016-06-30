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


#include "XQPController.hpp"

#include <rwlibs/algorithms/qpcontroller/QPSolver.hpp>
#include <rw/models/Device.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

using namespace rwlibs::algorithms;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::algorithms::qpcontroller;

using namespace boost::numeric::ublas;


XQPController::XQPController(Device::Ptr device,
							 Frame* controlFrame,
							 const State& state,
							 double dt):
	_device(device),
	_controlFrame(controlFrame),
	_state(state),
	_dt(dt),
	_dof(device->getDOF()),
	_space(BaseFrame),
	_P(identity_matrix<double>(6))
{
    _qlower = _device->getBounds().first;
    _qupper = _device->getBounds().second;

    _dqlimit = _device->getVelocityLimits();
    _ddqlimit = _device->getAccelerationLimits();
}

XQPController::~XQPController()
{
}

void XQPController::setAccScale(double scale) {
    _ddqlimit = scale*_device->getAccelerationLimits();
}

void XQPController::setVelScale(double scale) {
    _dqlimit = scale*_device->getVelocityLimits();
}


Q XQPController::inequalitySolve(const matrix<double>& G,
 				      			 const vector<double>& b,
 				      			 const vector<double>& lower,
 				      			 const vector<double>& upper,
 				      			 const std::list<Constraint>& constraints) {
    matrix<double> cmat = zero_matrix<double>(2*lower.size() + constraints.size(), lower.size());
    vector<double> limits(2*lower.size() + constraints.size());
    for (size_t i = 0; i<lower.size(); i++) {
        cmat(2*i,i) = 1;
        cmat(2*i+1,i) = -1;
        limits(2*i) = lower(i);
        limits(2*i+1) = -upper(i);
    }


    size_t index = 2*lower.size();
    for (std::list<Constraint>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
    	const Constraint& c = *it;
    	for (size_t i = 0; i<c._a.size(); i++) {
    		cmat(index, i) = c._a(i);
    	}
    	limits(index) = c._b;
    }


    //vector<double> qstart = (lower+upper)/2.0;
    vector<double> qstart(zero_vector<double>(lower.size()));
    //Try to start it with the minimal velocity
    for (size_t i = 0; i<qstart.size(); i++) {
        if (qstart(i) < lower(i))
            qstart(i) = lower(i);
        if (qstart(i) > upper(i))
            qstart(i) = upper(i);
    }
    for (std::list<Constraint>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
        const Constraint& c = *it;
        /*if (inner_prod(c._a.m(), lower) > c._b) {
            std::cout<<"Sets start to lower"<<std::endl;
            qstart = lower;
        } else if (inner_prod(c._a.m(), upper) > c._b) {
            qstart = upper;
            std::cout<<"Sets start to upper"<<std::endl;
        } else {*/
            for (size_t i = 0; i<qstart.size(); i++) {
                if (c._a(i) < 0)
                    qstart(i) = lower(i);
                else
                    qstart(i) = upper(i);
            }
            if (inner_prod(c._a.m(), qstart)< c._b) {
                std::cout<<"Could not find a valid starting velocity"<<std::endl;
                std::cout<<"a = "<<c._a<<std::endl;
                std::cout<<"b = "<<c._b<<std::endl;
                std::cout<<"lower = "<<lower<<std::endl;
                std::cout<<"upper = "<<upper<<std::endl;
                std::cout<<"qstart = "<<qstart<<std::endl;
            }
          //  std::cout<<"None of the bounds works to start it"<<std::endl;
        //}
    }





    QPSolver::Status status;
    vector<double> res = QPSolver::inequalitySolve(G, -1*b, cmat, limits, qstart, status);
    if (status == QPSolver::SUBOPTIMAL)
        std::cout<<"Solution appears to be suboptimal"<<std::endl;
    if (status == QPSolver::FAILURE)
        std::cout<<"Solution appears to be invalid"<<std::endl;

    return Q(res);
}




void XQPController::calculateVelocityLimits(vector<double>& lower,
                                            vector<double>& upper,
                                            const Q& q,
                                            const Q& dq)
{
    Q joint_pos = q;
    Q joint_vel = dq;
    double accmin, accmax, velmin, velmax, posmin, posmax;
    double x;
    for (size_t i = 0; i<_dof; i++) {
        //For the acceleration
        accmin = _dt*(-_ddqlimit)[i]+joint_vel(i);
        accmax = _dt*_ddqlimit[i]+joint_vel(i);
        //For the velocity
        velmin = (-_dqlimit)[i];
        velmax = _dqlimit[i];
        //For the position
        //If v_current<=v_max(X)
        x = _qupper[i]-joint_pos(i);
        if (x<=0) {
            posmax = 0;
        } else {
            //For qmax
            double j_x = Math::round(sqrt(1-8*x/(_dt*_dt*(-_ddqlimit)[i]))/2-1);
            double q_end_x = (x+_dt*_dt*(-_ddqlimit)[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_max_x = q_end_x-j_x*(-_ddqlimit)[i]*_dt;
            double X = x-_dt*q_max_x;
            if (X<=0){
                posmax = 0;
            } else {
                double j_X = Math::round(sqrt(1.-8*X/(_dt*_dt*(-_ddqlimit)[i]))/2.-1);
                double q_end_X = (X+_dt*_dt*(-_ddqlimit)[i]*(j_X*(j_X+1))/2)/(_dt*(j_X+1));
                posmax = q_end_X-j_X*(-_ddqlimit)[i]*_dt;
            }
        }
        x = joint_pos(i)-_qlower[i];
        if (x<=0)    {
            posmin = 0;
        } else {//For qmin
            double j_x = Math::round(sqrt(1+8*x/(_dt*_dt*_ddqlimit[i]))/2-1);
            double q_end_x = (-x+_dt*_dt*_ddqlimit[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_min_x = q_end_x-j_x*_ddqlimit[i]*_dt;
            double X = x+_dt*q_min_x;
            if (X<=0) {
                posmin = 0;
            } else {
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

Q XQPController::solve(const rw::math::Q& q,
                       const rw::math::Q& dq,
					   const rw::math::VelocityScrew6D<>& tcpscrew,
					   const std::list<Constraint>& constraints) {

    if (q.size() != _dof)
        RW_THROW("Length of input configuration does not match the Device");

    /*(6);
    Vector3D<> linvel = tcpscrew.linear();
    EAA<> angvel = tcpscrew.angular();

    tcp_vel(0) = linvel(0);
    tcp_vel(1) = linvel(1);
    tcp_vel(2) = linvel(2);
    tcp_vel(3) = angvel.angle()*angvel.axis()(0);
    tcp_vel(4) = angvel.angle()*angvel.axis()(1);
    tcp_vel(5) = angvel.angle()*angvel.axis()(2);
    */

    _device->setQ(q, _state);
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

    vector<double> tcp_vel = prod(P, tcpscrew.m());
	matrix<double> jac = prod(P, _device->baseJframe(_controlFrame,_state).m());

    //trim jacobian to remove world z-rotation
    //    matrix_range<matrix<double> > jac(jac6, range(0,6), range(0, jac6.size2()));

    matrix<double> A = prod(trans(jac),jac);
    vector<double> b = prod(trans(jac),tcp_vel);

    vector<double> lower(_dof);
    vector<double> upper(_dof);

    calculateVelocityLimits(lower, upper, q, dq);
    Q sol1 = inequalitySolve(A, b, lower, upper, constraints);

    return sol1;

}

void XQPController::setProjection(const matrix<double>& P, ProjectionFrame space) {
    _P = P;
    _space = space;
}
