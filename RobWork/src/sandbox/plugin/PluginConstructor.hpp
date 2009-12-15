#ifndef RW_PLUGIN_PLUGINCONSTRUCTOR_HPP
#define RW_PLUGIN_PLUGINCONSTRUCTOR_HPP

#include <rw/common/Ptr.hpp>

#include <string>

namespace rw {
namespace plugin {

class PluginConstructor
{
public:
    PluginConstructor(void);
    ~PluginConstructor(void);
 
    virtual std::string identifier() = 0;


    void add();

private:
    

};


typedef rw::common::Ptr<PluginConstructor> PluginConstructorPtr;

} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINCONSTRUCTOR_HPP