#include "SyncVelocityRamp.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::sandbox;

SyncVelocityRamp::SyncVelocityRamp(Device* device):
    _taus(Q::zero(device->getDOF())),
    _qstart(Q::zero(device->getDOF())),
    _qend(Q::zero(device->getDOF())),
    _maxtime(0)
{
    _vellimits = device->getVelocityLimits();
    _acclimits = device->getAccelerationLimits();
    
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

    calcRamp();
    
    /*Q delta = qend-qstart;
    double t = calcMaxTime(delta);
    _maxtime = t;
    std::cout<<"t = "<<t<<std::endl;
    std::cout<<"_vellimits "<<_vellimits<<std::endl;
    std::cout<<"_acclimits "<<_acclimits<<std::endl;
    std::cout<<"delta = "<<delta<<std::endl;    
    for (size_t i = 0; i<qstart.size(); i++) {
        std::cout<<"i "<<i<<std::endl;
        double dis = Math::sqr(_acclimits(i)*t) - 4*_acclimits(i)*fabs(delta(i));
        if (dis < 0)
            if (dis > -1e-9)
                dis = 0;
            else  {
                std::cout<<"dis = "<<dis<<std::endl;
                RW_THROW("Impossible trajectory");
            }

        if (fabs(delta(i)) > 0) {
            double tau1 = (_acclimits(i)*t + sqrt(dis))/(2*_acclimits(i));
            double tau2 = (_acclimits(i)*t - sqrt(dis))/(2*_acclimits(i));
            std::cout<<"tau1 = "<<tau1<<" tau2 = "<<tau2<<std::endl;
            if (tau2 > 0) //is tau2>0 it will be the solution we are looking for
                _taus(i) = tau2;
            else
                _taus(i) = tau1;
        } else {
            _taus(i) = 0;
        }
    }    
    std::cout<<"taus ="<<_taus<<std::endl;*/
}

double SyncVelocityRamp::duration() {
    return _duration;
}

/*
Q SyncVelocityRamp::getVelocities() {
    Q result(_acclimits.size());
    for (size_t i = 0; i<result.size(); i++) {
        result(i) = _taus(i)*_acclimits(i);
    }
    return result;        
}

Q SyncVelocityRamp::getAccelerations() {
    return _acclimits;
}
*/

/*
Q SyncVelocityRamp::x(double t) {
    if (t > _maxtime)
        return _end;
    if (t <= 0)
        return _start;
    
    Q result(_acclimits.size());
    for (size_t i = 0; i<result.size(); i++) {
        double sgn = Math::sign(_end(i) - _start(i));
        if (t<_taus(i)) {
            result(i) = _start(i) + sgn*t*t/2*_acclimits(i);
        } else if (t>_maxtime-_taus(i)) {
            double tau = _maxtime-t;
            result(i) = _end(i) - sgn*tau*tau/2*_acclimits(i);
        } else {
            double delta1 = sgn*_taus(i)*_taus(i)/2*_acclimits(i);
            double delta2 = sgn*_taus(i)*_acclimits(i)*(t-_taus(i));
            result(i) = _start(i) + delta1 + delta2;
        }        
    }
    return result;
}

*/






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
        if (dq != 0) {
            _wmax = std::min(_wmax, _vellimits(i)/dq);
            _dwmax = std::min(_dwmax, _acclimits(i)/dq);
        }
    }
    std::cout<<"wmax = "<<_wmax<<std::endl;
    std::cout<<"dwmax = "<<_dwmax<<std::endl;
    
    
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

Q SyncVelocityRamp::getVelocities() {
    return _wmax*(_qend-_qstart);
}

Q SyncVelocityRamp::getAccelerations() {
    return _dwmax*(_qend-_qstart);
}

double SyncVelocityRamp::getStartAccTime() {
    return _tau_s;
}

double SyncVelocityRamp::getEndAccTime() {
    return _tau_e;
}

double SyncVelocityRamp::s(double t) {
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


