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


#ifndef RW_GRASPPLANNING_CONTACTVALIDATEFILTER_HPP_
#define RW_GRASPPLANNING_CONTACTVALIDATEFILTER_HPP_

namespace rw { namespace sensor { class Contact3D; } }

namespace rw {
namespace graspplanning {

/**
 * @brief tests if a contact is valid in respect to some criterias implemented
 * by a sub class.
 */
class ContactValidateFilter {
public:

    /**
     * @brief destructor
     */
    virtual ~ContactValidateFilter(){};

    /**
     * @brief test if a contact \b contact is valid in regard to the criterias
     * of the class that implements this function.
     * @param contact [in] 3d contact
     * @return true if valid, false otherwise
     */
    virtual bool isValid(const rw::sensor::Contact3D& contact) = 0;

};
}
}
#endif
