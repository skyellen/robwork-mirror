#include <iostream>

#include <sandbox/dynload/DynamicLibraryLoader.hpp>
#include "Plugin.hpp"


/**
 * This program demonstrates the use of the DynamicLibraryLoader to
 * load in libraries
 */
int main(int argc, char** argv) {
    //Load PluginA  
    rw::common::DynamicLibraryLoader<Plugin> factoryA("libPluginA", "factory0");
    Plugin* pluginA = factoryA.get();
	if (pluginA != NULL)
		std::cout<<"PluginA = "<<pluginA->name()<<std::endl;

	//Load PluginB  
    rw::common::DynamicLibraryLoader<Plugin> factoryB("libPluginB", "factory0");
    Plugin* pluginB = factoryB.get();
	if (pluginB != NULL)
		std::cout<<"PluginB = "<<pluginB->name()<<std::endl;
		
    return 0;
   
}
