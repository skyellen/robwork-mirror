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

#include "ApproachMeasure3D.hpp"
#include "Grasp3D.hpp"

using namespace rw::graspplanning;
using namespace rw::math;

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



