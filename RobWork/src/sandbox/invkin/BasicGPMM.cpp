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


#include "BasicGPMM.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>

#include <boost/numeric/ublas/matrix.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwlibs::algorithms;

using namespace boost::numeric::ublas;

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


BasicGPMM::BasicGPMM(const rw::models::TreeDevice* device,
          const rw::kinematics::State& state,
          const rw::math::Q& qhome,
	          double dt):
    _device(device),
    _foi(device->getEnds()),
    _fkranges( createFKRanges(device->getBase(),_foi,state) ),
    _state(state),
    _qhome(qhome),
    _dof(device->getDOF()),
    _dt(dt),
    _P(identity_matrix<double>(6*_foi.size())),
    _space(BaseFrame)
{
    _jacCalc = _device->baseJCframes( _foi, state );

    _qlower = _device->getBounds().first;
    _qupper= _device->getBounds().second;
    _dqlimit = _device->getVelocityLimits();
    _ddqlimit = _device->getAccelerationLimits();

    _useJointLimitsCost = true;
    _useSingularityCost = false;

    _weightJointLimits = 1.0;
    _weightSingularity = 1.0;

    setJointLimitThreshold(0.2, 0.2); //default take 20% of the limit in each side
}

BasicGPMM::BasicGPMM(const rw::models::JointDevice* device,
		  const std::vector<rw::kinematics::Frame*>& foi,
		  const rw::kinematics::State& state,
		  const rw::math::Q& qhome,
		  double dt):
			_device(device),
			_foi(foi),
			_fkranges( createFKRanges(device->getBase(),_foi,state) ),
			_state(state),
			_qhome(qhome),
			_dof(device->getDOF()),
			_dt(dt),
			_P(identity_matrix<double>(6*_foi.size())),
			_space(BaseFrame)
{
     _jacCalc = _device->baseJCframes( _foi, state );

     _qlower = _device->getBounds().first;
     _qupper= _device->getBounds().second;
     _dqlimit = _device->getVelocityLimits();
     _ddqlimit = _device->getAccelerationLimits();

     _useJointLimitsCost = true;
     _useSingularityCost = false;

     _weightJointLimits = 1.0;
     _weightSingularity = 1.0;

     setJointLimitThreshold(0.2, 0.2); //default take 20% of the limit in each side
}


BasicGPMM::~BasicGPMM() {}

Q BasicGPMM::solve(const Q& q, const Q& dq, const std::vector<rw::math::VelocityScrew6D<> >& tcpvellist)
{
    RW_ASSERT((int)q.size() == _dof) ;
    RW_ASSERT((int)dq.size() == _dof);
    RW_ASSERT(tcpvellist.size() == _foi.size());
    _device->setQ(q, _state);

    //Calculate the right projection depending on whether it is in the base
    //frame or the frame of control
    //std::cout << "Settingup matrices" << std::endl;
    matrix<double> P;
    int dim = _foi.size();
    if (_space == ControlFrame) {
    	matrix<double> R = zero_matrix<double>(6*dim,6*dim);
    	for(size_t i=0;i<_foi.size();i++){
			Rotation3D<> rot = inverse(_device->baseTframe(_foi[i], _state).R());
			int offset=i*6;
			matrix_range<matrix<double> > rot1(R, range(0+offset,3+offset), range(0+offset,3+offset));
			rot1 = rot.m();
			matrix_range<matrix<double> > rot2(R, range(3+offset,6+offset), range(3+offset,6+offset));
			rot2 = rot.m();
    	}
    	P = prod(_P,R);
    } else {
        P = _P;
    }
    //std::cout << "create tcpscrew thing" << std::endl;
    // now create the tcpscrew matrix
    vector<double> tcpscrew(6*dim);
    for(int i=0;i<dim;i++){
    	const vector<double>& screw = tcpvellist[i].m();
    	for(int j=0;j<6;j++)
    		tcpscrew(j+i*6) = screw(j);
    }

    //std::cout << "Clculate jacobian and stuff" << std::endl;
    //matrix<double> jac = prod(P, _device->baseJframe(_controlFrame, _state).m());
    matrix<double> jac = prod(P, _jacCalc->get(_state).m());
    vector<double> tcpvel = prod(P, tcpscrew);

    //std::cout << "inverse" << std::endl;
    matrix<double> jac_inv = LinearAlgebra::pseudoInverse(jac);
    //matrix<double> j_invTj_inv = prod(trans(jac_inv), jav);
    matrix<double> jac_ort = identity_matrix<double>(_dof) - prod(jac_inv, jac);
    //std::cout << "calculate cost gradient" << std::endl;
    vector<double> g = getCostGradient(q, jac);
    vector<double> res = prod(jac_inv, tcpvel) + prod(jac_ort, g);
    //std::cout << "apply the velocity constraints" << std::endl;
    Q qres = applyJointVelocityConstraint(q, dq, Q(res));
    return qres;
}

vector<double> BasicGPMM::getCostGradient(const Q& q, const matrix<double>& jac) {
    vector<double> g = zero_vector<double>(_dof);

    if (_useJointLimitsCost) {
        for (int i = 0; i<_dof; i++) {
            if (q(i) > _thresholdUpper(i))
                g(i) = _weightJointLimits *(q(i) - _qhome(i))/(_qupper(i) - _qlower(i));
            else if (q(i) < _thresholdLower(i))
                g(i) = _weightJointLimits *(q(i) - _qhome(i))/(_qupper(i) - _qlower(i));
            else
                g(i) = 0;
        }
    }


    if (_useSingularityCost) {
        matrix<double> jtj = prod(trans(jac), jac);
        State state(_state);
        _device->setQ(q, state);
        double det = LinearAlgebra::det(jtj);
        Q qt = q;
        double delta = 0.001; // WTF?
        for (int i = 0; i<_dof; i++) {
            qt(i) -= delta;
            _device->setQ(qt, state);

            //Jacobian jac1 = _device->baseJframe(_controlFrame, state);
            Jacobian jac1 = _jacCalc->get(state);


            qt(i) = q(i) + delta;
            _device->setQ(qt, state);
            //Jacobian jac2 = _device->baseJframe(_controlFrame, state);
            Jacobian jac2 = _jacCalc->get(state);

            matrix<double> jtj1 = prod(trans(jac1.m()), jac1.m());
            matrix<double> jtj2 = prod(trans(jac2.m()), jac2.m());

            double ddet =
                (LinearAlgebra::det(jtj2) - LinearAlgebra::det(jtj1))
                / (2*delta);

            g(i) += _weightSingularity / (det*det) * ddet;

            qt(i) = q(i);
        }
    }
    return g;
}

Q BasicGPMM::applyJointVelocityConstraint(const Q& q, const Q& dq, const Q& dqnew) {

    vector<double> lower(_dof);
    vector<double> upper(_dof);
    calculatePosAndVelLimits(lower, upper, q, dq);
    double scale = 1;
    for (int i = 0; i<_dof; i++) {
        if (dqnew(i) > upper(i))
            scale = std::max(scale, upper(i)/dqnew(i));
        if (dqnew(i) < lower(i))
            scale = std::max(scale, lower(i)/dqnew(i));
    }
    return dqnew*scale;
}

void BasicGPMM::calculatePosAndVelLimits(
    vector<double>& lower,
    vector<double>& upper,
    const Q& q,
    const Q& dq)
{
    Q joint_pos = q;
    Q joint_vel = dq;
    double velmin, velmax, posmin, posmax;
    double x;

    for (int i = 0; i < _dof; i++) {
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
        upper(i) = std::max(velmax,posmax);
        lower(i) = std::max(velmin,posmin);


        //Because of numerical uncertainties we need to test whether upper>lower.
        if (upper(i) < lower(i)) {
            lower(i) = upper(i);
        }
    }
}

void BasicGPMM::setJointLimitThreshold(double thresholdLowerRatio, double thresholdUpperRatio) {
    _thresholdLower = _qlower + thresholdLowerRatio*(_qupper-_qlower);
    _thresholdUpper = _qupper - thresholdUpperRatio*(_qupper-_qlower);
}



void BasicGPMM::setUseJointLimitsCost(bool use) {
    _useJointLimitsCost = use;
}

void BasicGPMM::setUseSingularityCost(bool use) {
    _useSingularityCost = use;
}

void BasicGPMM::setJointLimitsWeight(double w) {
    _weightJointLimits = w;
}

void BasicGPMM::setSingularityWeight(double w) {
    _weightSingularity = w;
}

void BasicGPMM::setProjection(const boost::numeric::ublas::matrix<double>& P, ProjectionFrame space) {
    _P = P;
    _space = space;
}
