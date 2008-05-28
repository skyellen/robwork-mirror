#ifndef RW_SANDBOX_SYNCVELOCITYRAMP_HPP
#define RW_SANDBOX_SYNCVELOCITYRAMP_HPP

#include <rw/models/Device.hpp>
#include <rw/math/Q.hpp>

namespace rw {
namespace sandbox {


class SyncVelocityRamp
{
public:
	SyncVelocityRamp(rw::models::Device* device);
	virtual ~SyncVelocityRamp();
	
	void setTarget(const rw::math::Q& qcurrent, const rw::math::Q& qtarget);
	
	
	double duration();

	rw::math::Q getVelocities();

	rw::math::Q getAccelerations();

	rw::math::Q x(double t);
private:
    rw::math::Q _vellimits;
    rw::math::Q _acclimits;
    rw::math::Q _taus;
    rw::math::Q _qstart;
    rw::math::Q _qend;
    double _maxtime;
    

    double _ws;
    double _wmax;
    double _dwmax;
    double _duration;
    double _tau_s;
    double _tau_e;
    
    double calcMaxTime(const rw::math::Q& dists);
    void calcRamp();
    
    

    double getStartAccTime();

    double getEndAccTime();

    double s(double t);

};

} //end namespace sandbox
} //end namespace rw

#endif /*RW_SANDBOX_SYNCVELOCITYRAMP_HPP*/
