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

#ifndef RW_ROBWORK_HPP
#define RW_ROBWORK_HPP

#include <RobWorkConfig.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/common/Log.hpp>
#include <rw/plugin/PluginRepository.hpp>

namespace rw {

/**
 * @brief RobWork instance which holds objects to be shared among multiple plugins
 *
 * A RobWork instance contains common objects which may be used by multiple plugins
 * which may originate from different shared libraries.
 */
class RobWork
{
public:
    /**
     * @brief Creates RobWork instance
     */
    RobWork(void);

    /**
     * @brief Closes all plugins, stream etc. hold by the RobWork instance before destruction
     */
    ~RobWork(void);

    /**
     * @brief Returns the PluginRepository 
     * @return PluginRepository
     */
    rw::plugin::PluginRepository& getPluginRepository() {
        return _pluginRepository;
    }

    /**
     * @brief Returns the common log
     */
    rw::common::Log& getLog() {
        return _log;
    }

    /**
     * @brief Returns the version of RobWork
     */
    std::string getVersion() const {
        return RW_VERSION;
    }


private:
    rw::plugin::PluginRepository _pluginRepository;

    rw::common::Log _log;
};

typedef rw::common::Ptr<RobWork> RobWorkPtr;

} //end namespace rw


#endif //#ifndef RW_ROBWORK_HPP
