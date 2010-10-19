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

#ifndef RW_GRASPPLANNING_POSEGENERATOR_HPP_
#define RW_GRASPPLANNING_POSEGENERATOR_HPP_

#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/sensor/Contact3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief generates candidate contact point sets for grasping a given object.
 *
 * The method used identify
 *
 */
class ContactGenerator {

    /**
     * @brief
     */
    ContactGenerator(const rw::geometry::IndexedTriMesh& obj, int nrOfContacts);

    /**
     * @brief destructor
     */
    virtual ~ContactGenerator(){};

    /**
     * @brief generates a contact set from some heuristic
     */
    std::vector<Contact3D> generateContactSet( );

    /**
     * @brief generate next contact
     * @return
     */
    Contact3D generateNext();

private:

    const rw::geometry::IndexedTriMesh& _obj;
    const int _nrOfContacts;

    ContactGenerator();
};

}
}
#endif /*POSEGENERATOR_HPP_*/

