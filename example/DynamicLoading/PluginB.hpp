#ifndef PLUGINB_HPP_
#define PLUGINB_HPP_

#include "Plugin.hpp"
#include <iostream>
#include <rwlibs/dll/FactoryMacro.hpp>

class PluginB: public Plugin {
public:
    PluginB();
    
    ~PluginB();
    
    std::string name();    
};


#endif /*PLUGINB_HPP_*/
