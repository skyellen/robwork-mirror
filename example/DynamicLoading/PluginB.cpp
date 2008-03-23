#include "PluginB.hpp"


PluginB::PluginB() {
    std::cout<<"PluginB Constructor"<<std::endl;
}
    

PluginB::~PluginB()
{
}

std::string PluginB::name() {
    return "PluginB";
}

DLL_FACTORY_METHOD(PluginB);    

