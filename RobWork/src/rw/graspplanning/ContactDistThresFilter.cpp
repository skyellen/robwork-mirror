/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#include "ContactDistThresFilter.hpp"

#include <rw/math/MetricUtil.hpp>

using namespace rw::math;
using namespace rw::graspplanning;
using namespace rw::sensor;

bool ContactDistThresFilter::isContactPairValid(const Contact3D& c1, const Contact3D& c2){
    double dist = MetricUtil::dist2(c1.p, c2.p);
    if( dist>_maxDist ){
        std::cout << "maxDist: " << _maxDist << "  " << dist << std::endl;
        return false;
    }
    if( dist<_minDist ){
        std::cout << "minDist: " << _minDist << "  " << dist << std::endl;
        if(!_allowCloseWhenOpposite){
            return false;
        }
        // still valid if the contact normals are opposite
        double ndist = MetricUtil::dist2(normalize(c1.n),normalize(c2.n));
        if(ndist<1.5)
            return false;
    }
    return true;
}


bool ContactDistThresFilter::isValid(const Grasp3D& grasp){
    for(int i=0; i< ((int)grasp.contacts.size()) - 1; i++){
        for(size_t j=i+1; j<grasp.contacts.size(); j++){
            if( !isContactPairValid(grasp.contacts[i], grasp.contacts[j])){
                return false;
            }
        }
    }
    return true;
}
