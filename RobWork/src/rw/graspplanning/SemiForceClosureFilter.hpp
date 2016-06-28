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

#ifndef RW_GRASPPLANNING_SEMIFORCECLOSUREFILTER_HPP_
#define RW_GRASPPLANNING_SEMIFORCECLOSUREFILTER_HPP_

#include "GraspValidateFilter.hpp"

#include <cstddef>

namespace rw {
namespace graspplanning {

/**
 * @brief A conservative estimate of the force closure properties of the grasp are
 * used to indicate weather a grasp is valid or not.
 *
 * The method is described in "Grasping the Dice by Dicing the Grasp"
 */
class SemiForceClosureFilter: public GraspValidateFilter {
public:

    SemiForceClosureFilter(size_t nrContacts):
        _nrContacts(nrContacts),_avgScale(1.0/nrContacts){};

    virtual ~SemiForceClosureFilter(){};

    bool isValid(const Grasp3D& grasp);

private:
    size_t _nrContacts;
    double _avgScale;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
