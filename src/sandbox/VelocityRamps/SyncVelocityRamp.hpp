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

	rw::math::Q getMaxVelocities();

	rw::math::Q getMaxAccelerations();

	rw::math::Q x(double t);
private:
    rw::math::Q _vellimits;
    rw::math::Q _acclimits;
    rw::math::Q _taus;
    rw::math::Q _start;
    rw::math::Q _end;
    double _maxtime;
    
    double calcMaxTime(const rw::math::Q& dists);
};

} //end namespace sandbox
} //end namespace rw

#endif /*RW_SANDBOX_SYNCVELOCITYRAMP_HPP*/
