#include "ApproachMeasure3D.hpp"

#include <boost/foreach.hpp>
#include <rw/math/MetricUtil.hpp>

using namespace rw::graspplanning;
using namespace rw::math;
using namespace rw::sensor;

double ApproachMeasure3D::quality(const Grasp3D& grasp) const {
    double quality = 0;
    for(size_t i=0;i<grasp.approach.size();i++){
        const Vector3D<> &v1 = grasp.approach[i];
        const Vector3D<> &v2 = grasp.contacts[i].n;
        double ang = fabs( acos( dot(normalize(v1),normalize(v2)) ));
        if(ang>_maxAngle){
            return 0;
        }
        quality += 1-ang/_maxAngle;
    }
    return quality/grasp.approach.size();
}



