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

#ifndef RWSIM_SIMULATOR_ODEPLUGIN_HPP_
#define RWSIM_SIMULATOR_ODEPLUGIN_HPP_

#include <rw/common/Plugin.hpp>

namespace rwsim {
namespace simulator {

/**
 * @brief A ODE plugin that define extensions for rwsim.simulator.PhysicsEngine
 */
class ODEPlugin: public rw::common::Plugin {
public:

    /**
     * @brief constructor
     */
    ODEPlugin();

    //! destructor
    virtual ~ODEPlugin();

    //! @copydoc rw::common::Plugin::getExtensionDescriptors
    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

    //! @copydoc rw::common::Plugin::makeExtension
    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);

};

}
}

#endif /* ODEBODY_HPP_ */
