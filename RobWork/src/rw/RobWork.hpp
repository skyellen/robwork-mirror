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
#include <rw/common/PropertyMap.hpp>
#include <rw/common/ExtensionRegistry.hpp>

namespace rw {

/**
 * @brief RobWork instance which holds objects to be shared among multiple plugins
 *
 * A RobWork instance contains common objects and configuration which may be used
 * by multiple plugins which may originate from different shared libraries.
 */
class RobWork
{
public:
    //! @brief smart pointer type to this class
    typedef rw::common::Ptr<RobWork> Ptr;

    /**
     * @brief Creates RobWork instance
     */
    RobWork(void);

    /**
     * @brief Closes all plugins, stream etc. hold by the RobWork instance before destruction
     */
    ~RobWork(void);

    /**
     * @brief Returns the common log
     */
    rw::common::Log& getLog();

    /**
     * @brief get a pointer to the common log
     * @return
     */
	rw::common::Log::Ptr getLogPtr();

	/**
	 * @brief set logger for this instance of RobWork
	 * @param log
	 */
	void setLog(rw::common::Log::Ptr log);

    /**
     * @brief Returns the version of RobWork
     */
    std::string getVersion() const
    {
        return RW_VERSION;
    }

    /**
     * @brief initialize robwork
     *
     * Reads in its configuration file which specify plugins and so on.
     */
    void initialize(const std::vector<std::string>& plugins = std::vector<std::string>());

	/**
	 * @brief finalizes the robwork instance
	 */
	void finalize();

    /**
     * @brief get settings of RobWork instance
     * @return
     */
    rw::common::PropertyMap& getSettings();

    /**
     * @brief get the extension registry
     * @return
     */
    rw::common::Ptr<rw::common::ExtensionRegistry> getExtensionRegistry();

    /**
     * @brief set extension registry of this instance of robwork
     */
    void setExtensionRegistry(rw::common::Ptr<rw::common::ExtensionRegistry> extreg);

    /**
     * @brief Check if RobWork has been initialized.
     * @return true if initialized, false otherwise.
     */
    bool isInitialized() const;

    /**
     * @brief returns an RobWork instance
     */
    static RobWork::Ptr getInstance();

    /**
     * @brief initialize robwork
     */
    static void init();

    /**
     * @brief initialize robwork - including possible command line options
     */
    static void init(int argc, const char* const* argv);

	/**
	 * @brief finalize the robwork instance
	 */
	static void finish();

    /**
     * @brief sets the robwork instance
     * @param rw [in] the new instance
     */
    static void setInstance(RobWork::Ptr rw);

private:
    rw::common::PropertyMap _settings;
    std::string _settingsFile;
    std::map<std::string, std::time_t> _pluginChangedMap;
    bool _initialized;
};

} //end namespace rw


#endif //#ifndef RW_ROBWORK_HPP
