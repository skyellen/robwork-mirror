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
 
    virtual std::string identifier() const = 0;

    enum PluginType { DEVICE = 1, JOINT, FRAME, INVKIN_SOLVER, TRAJECTORY, USER }; 

    virtual PluginType getType() const = 0;

    void add();





private:
    

};


typedef rw::common::Ptr<PluginConstructor> PluginConstructorPtr;

} //end namespace plugin
} //end namespace rw

#endif //#ifndef RW_PLUGIN_PLUGINCONSTRUCTOR_HPP