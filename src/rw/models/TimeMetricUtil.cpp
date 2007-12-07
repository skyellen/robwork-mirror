#include "TimeMetricUtil.hpp"
#include <rw/math/MetricUtil.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;

//----------------------------------------------------------------------
// Time distances

double TimeMetricUtil::TimeDistance(const Q& from,
							   	    const Q& to,
							   	    const Q& velocity)
{
    RW_ASSERT(from.size() == to.size());
    RW_ASSERT(velocity.size() <= from.size());
    double result = 0;
    for (size_t i = 0; i < velocity.size(); i++) {                        
        result = std::max(result, fabs((from[i] - to[i]) / velocity[i]));
    }
    return result;    
}

double TimeMetricUtil::TimeDistance(const Q& from,
							   		const Q& to,
							   		const DeviceModel& device)
{
    return TimeDistance(from, to, device.getVelocityLimits());
}

double TimeMetricUtil::TimeDistance(const State& from,
							   		const State& to,
							   		const DeviceModel& device)
{
    return TimeDistance(device.getQ(from), device.getQ(to), device.getVelocityLimits());
}

double TimeMetricUtil::TimeDistance(const State& from,
							   		const State& to,
						   			const WorkCell& workcell)
{
    // We simply compute the time distance for each device and choose the
    // maximum value.

    const int len = workcell.getDevices().size();
    Q device_times(len);

    for (int i = 0; i < len; i++) {
        const DeviceModel& device = *workcell.getDevices()[i];
        device_times[i] = TimeDistance(from, to, device);
    }

    return MetricUtil::MaxLength(device_times);
}
