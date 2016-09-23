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

#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
#define RW_PLUGIN_PLUGINREPOSITORY_HPP

/**
 * @file rw/plugin/PluginRepository.hpp
 *
 * Defines the rw::plugin::PluginRepository
 */


#include "PluginFactory.hpp"

#include <boost/function.hpp>
#include <map>
#include <vector>




namespace rw {
namespace plugin {



/** @addtogroup plugin */
/*@{*/

/**
 * @brief The PluginRepository provides a container load methods for plugins
 *
 * The PluginRepository contain a collection of pointers rw::plugin::PluginFactoryBase instances
 * which can be used to create plugins. 
 *
 * The PluginRepository has a number of different methods to load plugins and methods for 
 * searching for a plugins either based on identifiers or by type.
 */
class PluginRepository
{
public:
	//! @brief Smart pointer type for PluginRepository.
    typedef rw::common::Ptr<PluginRepository> Ptr;

    /**
     * @brief Constructs an empty repository
     */
    PluginRepository() {};

    /**
     * @brief Destructor
     */
    ~PluginRepository() {}

    /**
     * @brief Loads in a PluginFactoryBase from a file.
     * 
     * If the file could not be loaded or does not contain an object of type
     * PluginFactoryBase a rw::common::Exception is thrown.
     *
     * @param filename [in] File to load
     */
    void load(const std::string& filename);

    /**
     * @brief Attempts to load all dll files in folder
     *
     * If finding a dll file which can not be loaded or does not contain an object of type
     * PluginFactoryBase a rw::common::Exception is thrown.
     * 
     * @param path [in] Path from which to attempt for load plugins
     * @param searchSubFolders [in] True to search recursively into subfolders.
     */
    void loadFilesInFolder(const std::string& path, bool searchSubFolders);

    /**
     * @brief this adds a plugin directly
     *
     * This is especially usefull if you want to add plugin functionality statically
     * avoiding dynamic loading or managing the dynamic loading your self.
     * @param plugin [in] the plugin factory base
     * @param force [in] if true, an existing plugin with same identifier string will be replaced
     * by the new \b plugin
     */
    void addPlugin(PluginFactoryBase::Ptr plugin, bool force=false);

    /**
     * @brief Add a listener which should be informed when new plugins are loaded
     *
     * @param listener [in] The function to call for notification
     */
    void addListener(boost::function<void(void)>& listener);

    /**
     * @brief Returns map in which keys are the identifiers of loaded plugins factories and the
     * value is the plugin factory.
     *
     * @return Const reference to std::map with identifier and PluginFactoryBase::Ptr.
     */
    const std::map<std::string, PluginFactoryBase::Ptr>& getAllPlugins() const;

    /**
     * @brief Returns map in which keys are the identifiers of loaded plugins factories and the
     * value is the plugin factory.
     *
     * @return Reference to std::map with identifier and PluginFactoryBase::Ptr.
     */
    std::map<std::string, PluginFactoryBase::Ptr>& getAllPlugins();

    /**
     * @brief Returns all rw::common::PluginFactory<T> instances which matches the template argument T
     *
     * @return List of all factories matching T 
     */
    template <class T>
    std::vector<rw::common::Ptr<PluginFactory<T> > > getPlugins() {
        std::vector<rw::common::Ptr<PluginFactory<T> > > result;
		
        for (std::map<std::string, PluginFactoryBase::Ptr>::iterator it = _str2constructorMap.begin(); it != _str2constructorMap.end(); ++it) {
            rw::common::Ptr<PluginFactory<T> > factory = (*it).second.cast<PluginFactory<T> >();
            if (factory != NULL)
                result.push_back(factory);

        }
        return result;
    }


private:


    std::map<std::string, PluginFactoryBase::Ptr> _str2constructorMap;

    std::vector<boost::function<void(void)> > _listeners;
};

/** @} */


} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
