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

#include "PluginRepository.hpp"

#include <rw/common/IOUtil.hpp>
#include "DynamicLibraryLoader.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::plugin;


//PluginRepository PluginRepository::_repository;


void PluginRepository::load(const std::string& filename) {
    PluginFactoryBasePtr constructor; 
    try 
    {
        DynamicLibraryLoader<PluginFactoryBase>* loader = new DynamicLibraryLoader<PluginFactoryBase>(filename);
        constructor = ownedPtr(loader->get());
    } catch (const Exception& exp) 
    {
        RW_THROW("Unable to load plugin: "<<filename<<". Failed with message: "<<exp.getMessage().getText());
    }

    if (constructor != NULL)  
    {
        const std::string id = constructor->identifier();
        if (_str2constructorMap.find(id) == _str2constructorMap.end()) {
            _str2constructorMap[id] = constructor;
            //Log::debugLog()<<"Loaded Plugin "<<id<<std::endl;
        } else {
            RW_THROW("A Plugin with identifier "<<id<<" has already been loaded!");
        }
    } else {
        RW_THROW("Unable to load plugin: "<<filename);
    }
}

void PluginRepository::addPlugin(PluginFactoryBase::Ptr plugin, bool force){
    const std::string id = plugin->identifier();
    if (force || (_str2constructorMap.find(id) == _str2constructorMap.end()) ) {
        _str2constructorMap[id] = plugin;
        //Log::debugLog()<<"Loaded Plugin "<<id<<std::endl;
    } else {
        RW_WARN("A Plugin with identifier "<<id<<" has already been loaded!");
    }
}

void PluginRepository::loadFilesInFolder(const std::string& path, bool searchSubFolders) {
    
    std::vector<std::string> files = IOUtil::getFilesInFolder(path, true, true, "*."+OS::getDLLExtension());
//    std::vector<std::string> files = IOUtil::getFilesInFolder(path, true, "*.dll");

    BOOST_FOREACH(std::string str, files) {
        //std::cout<<"DLL File = "<<str<<std::endl;
        load(str);        
    }
}

void PluginRepository::addListener(boost::function<void(void)>& listener) {
    _listeners.push_back(listener);
}   




/*std::vector<PluginFactoryBasePtr> PluginRepository::getPlugins(PluginType type) const {
    std::vector<PluginFactoryBasePtr> result;
    for (std::map<std::string, PluginFactoryBasePtr>::const_iterator it = _str2constructorMap.begin(); it != _str2constructorMap.end(); ++it) {
        if ((*it).second->getType() == type) {
            result.push_back((*it).second);
        }
    }
    return result;
}
*/

const std::map<std::string, PluginFactoryBasePtr>& PluginRepository::getAllPlugins() const {
    return _str2constructorMap;
}


std::map<std::string, PluginFactoryBasePtr>& PluginRepository::getAllPlugins() {
    return _str2constructorMap;
}



/*
PluginRepository& PluginRepository::instance() {
    static PluginRepository repository;    
    return repository;
}
*/
