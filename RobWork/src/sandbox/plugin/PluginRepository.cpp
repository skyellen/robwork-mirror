#include "PluginRepository.hpp"

#include <rw/common/IOUtil.hpp>
#include <rwlibs/dll/DynamicLibraryLoader.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::plugin;
using namespace rwlibs::dll;


//PluginRepository PluginRepository::_repository;


void PluginRepository::add(const std::string& filename) {
    PluginConstructorPtr constructor; 
    try 
    {
        DynamicLibraryLoader<PluginConstructor> loader(filename);
        constructor = loader.get();
    } catch (const Exception& exp) 
    {
        RW_THROW("Unable to load plugin: "<<filename<<". Failed with message: "<<exp.getMessage().getText());
    }

    if (constructor != NULL) 
    {
        const std::string id = constructor->identifier();
        if (_str2constructorMap.find(id) == _str2constructorMap.end()) {
            _str2constructorMap[id] = constructor;
            Log::debugLog()<<"Loaded Plugin "<<id<<std::endl;
        } else {
            RW_THROW("A Plugin with identifier "<<id<<" has already been loaded!");
        }
    } else {
        RW_THROW("Unable to load plugin: "<<filename);
    }
}

void PluginRepository::addFilesInFolder(const std::string& path) {
    
    std::vector<std::string> files = IOUtil::getFilesInFolder(path, true, "*.dll");

    BOOST_FOREACH(std::string str, files) {
        std::cout<<"DLL File = "<<str<<std::endl;
        add(str);        
    }

}

void PluginRepository::addListener(boost::function<void(void)>& listener) {
    _listeners.push_back(listener);
}   




std::vector<PluginConstructorPtr> PluginRepository::getPlugins(PluginConstructor::PluginType type) const {
    std::vector<PluginConstructorPtr> result;
    for (std::map<std::string, PluginConstructorPtr>::const_iterator it = _str2constructorMap.begin(); it != _str2constructorMap.end(); ++it) {
        if ((*it).second->getType() == type) {
            result.push_back((*it).second);
        }
    }
    return result;
}

const std::map<std::string, PluginConstructorPtr>& PluginRepository::getPlugins() const {
    return _str2constructorMap;
}


std::map<std::string, PluginConstructorPtr>& PluginRepository::getPlugins() {
    return _str2constructorMap;
}


PluginRepository& PluginRepository::instance() {
    static PluginRepository repository;    
    return repository;
}
