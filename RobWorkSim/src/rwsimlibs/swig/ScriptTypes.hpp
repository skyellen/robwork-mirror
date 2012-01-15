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

#ifndef RWSIM_SWIG_REMOTETYPES_HPP_
#define RWSIM_SWIG_REMOTETYPES_HPP_

#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/BodyUtil.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/control/BodyController.hpp>


#ifdef __cplusplus
extern "C" {
#endif
    int luaopen_rws(struct lua_State* L); // declare the wrapped module
#ifdef __cplusplus
}
#endif

namespace rwsim {
namespace swig {

    template <typename T>
    std::string toString(const T& x)
    {
        std::ostringstream buf;
        buf << x;
        return buf.str();
    }

    typedef rwsim::dynamics::DynamicWorkCell DynamicWorkCell;
    typedef rwsim::dynamics::Body Body;
    typedef rwsim::dynamics::RigidBody RigidBody;
    typedef rwsim::dynamics::FixedBody FixedBody;
    typedef rwsim::dynamics::KinematicBody KinematicBody;
    typedef rwsim::dynamics::BodyInfo BodyInfo;
    typedef rwsim::dynamics::DynamicDevice DynamicDevice;


    // for now we add all static functions here
    DynamicWorkCell* getDynamicWorkCell();

    /**
     * @brief set current robworkstudio instance
     */
    void setDynamicWorkCell(DynamicWorkCell* dwc);

    /// These functions all work on the current robworkstudio state

}
}

#endif /* REMOTETYPES_HPP_ */
