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


#ifndef RW_MODELS_DEPENDENTJOINT_HPP
#define RW_MODELS_DEPENDENTJOINT_HPP

#include <rw/models/Joint.hpp>

namespace rw {
namespace models {

/**
 * @brief Dependent joints are 0-dof joints for which the actual joints transformation depends on one of more other joints.
 *
 * DependentJoint is an abstract class from which all dependent joints should inherit.
 */
class DependentJoint: public Joint
{
public:

    /**
     * @brief Destructor
     */
    virtual ~DependentJoint();

    /**
     * @brief Returns true if the DependentJoint is controlled by \b joint.
     *
     * A DependentJoint may depend on more than one joints.
     *
     * @param joint [in] Joints to test with
     * @return True if this is controlled by \b joint
     *
     */
    virtual bool isControlledBy(const Joint* joint) const = 0;

protected:

    /**
     * @brief Constructs DependentJoint
     * @param name [in] Name of the joints
     */
    DependentJoint(const std::string& name);
};

} //end namespace models
} //end namespace rw

#endif //end include guard
