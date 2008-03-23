#include "PluginA.hpp"

PluginA::PluginA() {
    std::cout<<"PluginA Constructor"<<std::endl;
}


PluginA::~PluginA()
{
}

std::string PluginA::name() {
    return "PluginA";
}

DLL_FACTORY_METHOD(PluginA);
