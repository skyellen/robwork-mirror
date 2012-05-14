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

#ifndef RW_GRASPPLANNING_GRASP2D_HPP_
#define RW_GRASPPLANNING_GRASP2D_HPP_

#include <rw/sensor/Contact2D.hpp>
#include <rw/math/Vector2D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief a grasp is a set of contacts between the object to be grasped and 
 * the robot gripper.
 */

class Grasp2D {
public:

    Grasp2D(int nrOfContacts):
        contacts(nrOfContacts),
        approach(nrOfContacts)
    {}
    
    void scale(double clerance){
        for(int i=0;i<3;i++)
            contacts[i].p += normalize(approach[i])*clerance;
    }
    
    double phi,psi;
    rw::math::Vector2D<> center;
    std::vector<rw::sensor::Contact2D> contacts;
    std::vector<rw::math::Vector2D<> > approach;
    
};
}
}

#endif /*GRASP_HPP_*/
