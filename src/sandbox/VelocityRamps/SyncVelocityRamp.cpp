#include "SyncVelocityRamp.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::sandbox;

SyncVelocityRamp::SyncVelocityRamp(Device* device):
    _taus(Q::zero(device->getDOF())),
    _start(Q::zero(device->getDOF())),
    _end(Q::zero(device->getDOF())),
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
    Q delta = qend-qstart;
    _start = qstart;
    _end = qend;
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
    std::cout<<"taus ="<<_taus<<std::endl;
}

double SyncVelocityRamp::duration() {
    return _maxtime;
}

Q SyncVelocityRamp::getMaxVelocities() {
    Q result(_acclimits.size());
    for (size_t i = 0; i<result.size(); i++) {
        result(i) = _taus(i)*_acclimits(i);
    }
    return result;        
}

Q SyncVelocityRamp::getMaxAccelerations() {
    return _acclimits;
}



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


