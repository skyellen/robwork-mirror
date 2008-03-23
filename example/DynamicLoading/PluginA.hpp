#ifndef PLUGINA_HPP_
#define PLUGINA_HPP_

#include <sandbox/dynload/FactoryMacro.hpp>

#include "Plugin.hpp"
#include <string>
#include <iostream>

class PluginA: public Plugin {
public:
    PluginA();
    
    ~PluginA();
    
    std::string name();
    
      
};


#endif /*PLUGINA_HPP_*/
