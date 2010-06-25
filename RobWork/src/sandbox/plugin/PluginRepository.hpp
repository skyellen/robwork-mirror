#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP
#define RW_PLUGIN_PLUGINREPOSITORY_HPP


#include "PluginConstructor.hpp"

#include <boost/function.hpp>
#include <map>
#include <vector>




namespace rw {
namespace plugin {

    
class PluginRepository
{
public:
    PluginRepository() {};

    ~PluginRepository() {} 
    
    void add(const std::string& filename);

    void addFilesInFolder(const std::string& path);

    void addListener(boost::function<void(void)>& listener);

    std::vector<PluginConstructorPtr> getPlugins(PluginConstructor::PluginType type) const;

    const std::map<std::string, PluginConstructorPtr>& getPlugins() const;
    std::map<std::string, PluginConstructorPtr>& getPlugins();

    static PluginRepository& instance();

    

private:
    

    std::map<std::string, PluginConstructorPtr> _str2constructorMap;    

    std::vector<boost::function<void(void)> > _listeners;    
};


} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINREPOSITORY_HPP