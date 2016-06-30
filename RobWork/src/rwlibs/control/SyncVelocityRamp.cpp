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


#include "SyncVelocityRamp.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Math.hpp>
#include <rw/models/Device.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::control;

SyncVelocityRamp::SyncVelocityRamp(Device* device):
    _taus(Q::zero(device->getDOF())),
    _qstart(Q::zero(device->getDOF())),
    _qend(Q::zero(device->getDOF()))
{
    _vellimits = device->getVelocityLimits();
    _acclimits = device->getAccelerationLimits();

}

SyncVelocityRamp::SyncVelocityRamp(const rw::math::Q& vellimits, const rw::math::Q& acclimits):
    _vellimits(vellimits),
    _acclimits(acclimits),
    _taus(Q::zero(vellimits.size())),
    _qstart(Q::zero(vellimits.size())),
    _qend(Q::zero(vellimits.size()))
{
}

SyncVelocityRamp::~SyncVelocityRamp()
{
}


double SyncVelocityRamp::calcMaxTime(const Q& dists) {
    RW_ASSERT(dists.size() == _vellimits.size());

    double maxtime = 0;
    for (size_t i = 0; i<dists.size(); i++) {
        double t = 0;
        double tau = _vellimits(i)/_acclimits(i);
        double eps = sqrt(fabs(dists(i))/(_acclimits(i)));
        std::cout<<"tau = "<<tau<<std::endl;
        std::cout<<"eps = "<<eps<<std::endl;
        if (eps<tau)
            t = 2*eps;
        else {
            double dtau = tau*_vellimits(i);
            double T = (fabs(dists(i))-dtau)/_vellimits(i);
            t = T+2*tau;
        }
        maxtime = std::max(t, maxtime);
    }
    return maxtime;
}

void SyncVelocityRamp::setTarget(const rw::math::Q& qstart, const rw::math::Q& qend) {
    _qstart = qstart;
    _qend = qend;
    _ws = 0;
    calcRamp();
}





double SyncVelocityRamp::duration() {
    return _duration;
}

void SyncVelocityRamp::calcRamp() {
    _wmax = 1e10;
    _dwmax = 1e10;
    _ws = 0;

    if (_qend == _qstart) {
        _wmax = 0;
        _dwmax = 0;
        _duration = 0;
        _tau_s = 0;
        _tau_e = 0;
        return;
    }

    for (size_t i = 0; i<_qstart.size(); i++) {
        double dq = fabs(_qend(i)-_qstart(i));
        //std::cout<<"dq = "<<dq<<std::endl;
        if (dq != 0) {
            _wmax = std::min(_wmax, _vellimits(i)/dq);
            _dwmax = std::min(_dwmax, _acclimits(i)/dq);
        }
    }
    //std::cout<<"wmax = "<<_wmax<<std::endl;
    //std::cout<<"dwmax = "<<_dwmax<<std::endl;


    //Compare using eq. 13 to see which of the cases
    double lhs = sqrt(2*Math::sqr(_ws/_dwmax)+4/_dwmax);
    double rhs = 2*_wmax/_dwmax;
    if (lhs <= rhs) {
        _duration = _ws/_wmax+sqrt(2*Math::sqr(_ws/_dwmax)+4/_dwmax);
        _tau_s = _duration/2-_ws/(2*_dwmax);
        _tau_e = _duration/2+_ws/(2*_dwmax);
    } else {
        _duration = 1/_wmax+(Math::sqr(_wmax-_ws)+Math::sqr(_wmax))/(2*_wmax*_dwmax);
        _tau_s = (_wmax-_ws)/_dwmax;
        _tau_e = (_wmax)/_dwmax;
    }
}


Q SyncVelocityRamp::x(double t) {
    return _qstart+s(t)*(_qend-_qstart);
}

Q SyncVelocityRamp::dx(double t) {
    return ds(t)*(_qend-_qstart);
}

Q SyncVelocityRamp::ddx(double t) {
    return dds(t)*(_qend-_qstart);
}

Q SyncVelocityRamp::getExtremumVelocities() {
    return _wmax*(_qend-_qstart);
}

Q SyncVelocityRamp::getExtremumAccelerations() {
    return _dwmax*(_qend-_qstart);
}

double SyncVelocityRamp::getStartAccTime() {
    return _tau_s;
}

double SyncVelocityRamp::getEndAccTime() {
    return _tau_e;
}

double SyncVelocityRamp::s(double t) {
    if (t<0)
        t = 0;

    double tau1 = std::min(t, _tau_s);
    double s1 = _ws*t+0.5*_dwmax*tau1*tau1;
    if (t<_tau_s)
        return s1;

    double tau2 = std::min(t, _duration-_tau_e);
    double s2 = _wmax*(tau2-_tau_s) + s1;
    if (t<=_duration-_tau_e)
        return s2;

    if (t<_duration) {
        double delta = (_duration-_tau_e);
        double tpart = _tau_e*_dwmax*t + 0.5*_dwmax*t*t-_dwmax*_duration*t;
        double cpart = _tau_e*_dwmax*delta + 0.5*_dwmax*delta*delta - _dwmax*_duration*delta;
        return s2 - (tpart - cpart)+_tau_e*_dwmax*t-_tau_e*_dwmax*delta;
    }
    return 1;
}

double SyncVelocityRamp::ds(double t) {
    if (t<0)
        t = 0;

    double tau1 = std::min(t, _tau_s);
    double ds1 = _ws+_dwmax*tau1;
    if (t<_tau_s)
        return ds1;

    // double tau2 = std::min(t, _duration-_tau_e);
    double ds2 = ds1;
    if (t<=_duration-_tau_e)
        return ds2;

    if (t<_duration) {
        double delta = t-(_duration-_tau_e);
        double ds3 = ds2 - _dwmax*delta;
        return ds3;
    }
    return 0;
}

double SyncVelocityRamp::dds(double t) {
    if (t<_tau_s)
        return _dwmax;
    if (t<_duration-_tau_e)
        return 0;
    if (t<_duration)
        return -_dwmax;
    return 0;
}


