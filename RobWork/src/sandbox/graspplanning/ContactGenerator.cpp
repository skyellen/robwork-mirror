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


#include "ContactGenerator.hpp"




ContactGenerator::ContactGenerator(const IndexedTriMesh& obj, int nrOfContacts):
    _obj(obj),_nrOfContacts(nrOfContacts)

{
    // calculate object center
    // calculate object bounding sphere

}

std::vector<Contact> ContactGenerator::generateContactSet()
{
    // 1. generate random point "p" inside sphere, close to center point
    // 2. generate two orthogonal vectors from "p" that span some plane
    // 3. shoot N rays from p such that each ray is displaysed
    //    with a small angle to the plane and
    //    such that an angle of about 360/N exist between the vectors when projected to the plane


}
