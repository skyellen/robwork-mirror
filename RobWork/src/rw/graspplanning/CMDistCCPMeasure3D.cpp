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

#include "CMDistCCPMeasure3D.hpp"
#include "Grasp3D.hpp"

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
    Vector3D<> CCP = sum/((double)grasp.contacts.size());
    const double dist = MetricUtil::dist2(_CM, CCP);
    if(dist>_maxDist)
        return 0;
    return (_maxDist-dist)/_maxDist;
}



