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


#ifndef RW_GRASPPLANNING_GRASP3D_HPP_
#define RW_GRASPPLANNING_GRASP3D_HPP_

#include <rw/sensor/Contact3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace graspplanning {

    /**
     * @brief a grasp is a set of contacts between the object to be grasped and
     * the robot gripper.
     */
    class Grasp3D {
    public:

        Grasp3D(int nrOfContacts=1):
            contacts(nrOfContacts),
            approach(nrOfContacts)
        {}

        Grasp3D(const std::vector<rw::sensor::Contact3D> cons):
            contacts(cons),
            approach(cons.size())
        {}

        void scale(double clerance){
            for(size_t i=0;i<contacts.size();i++)
                contacts[i].p += normalize(approach[i])*clerance;
        }

        double phi,psi,quality;
        std::vector<rw::sensor::Contact3D> contacts;
        std::vector<rw::math::Vector3D<> > approach;
        rw::math::Vector3D<> center;


    };
}
}
#endif /*GRASP_HPP_*/
