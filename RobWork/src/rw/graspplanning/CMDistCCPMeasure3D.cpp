#include "CMDistCCPMeasure3D.hpp"

#include <boost/foreach.hpp>
#include <rw/math/MetricUtil.hpp>

using namespace rw::math;
using namespace rw::graspplanning;
using namespace rw::sensor;

double CMDistCCPMeasure3D::quality(const Grasp3D& grasp) const {
    Vector3D<> sum(0,0,0);
    BOOST_FOREACH(const Contact3D& con, grasp.contacts){
        sum += con.p;
    }
    Vector3D<> CCP = sum/grasp.contacts.size();
    const double dist = MetricUtil::dist2(_CM, CCP);
    if(dist>_maxDist)
        return 0;
    return (_maxDist-dist)/_maxDist;
}



