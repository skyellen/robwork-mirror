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

#include "ContactDistThresFilter.hpp"
#include "Grasp3D.hpp"

#include <rw/math/MetricUtil.hpp>
#include <rw/sensor/Contact3D.hpp>

using namespace rw::math;
using namespace rw::graspplanning;
using namespace rw::sensor;

bool ContactDistThresFilter::isContactPairValid(const Contact3D& c1, const Contact3D& c2){
    double dist = MetricUtil::dist2(c1.p, c2.p);
    if( dist>_maxDist ){
        return false;
    }
    if( dist<_minDist ){
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
