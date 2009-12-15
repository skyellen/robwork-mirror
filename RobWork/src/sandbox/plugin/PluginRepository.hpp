#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
#define RW_PLUGIN_PLUGINREPOSITORY_HPP


#include "PluginConstructor.hpp"
#include <map>

namespace rw {
namespace plugin {

    
class PluginRepository
{
public:
    ~PluginRepository() {} 

    void add(const std::string& filename);

    static PluginRepository& instance();

private:
    PluginRepository() {};

    std::map<std::string, PluginConstructorPtr> _str2constructorMap;

    static PluginRepository _repository;    
};


} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP