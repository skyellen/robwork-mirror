#include <iostream>

#include <rwlibs/dll/DynamicLibraryLoader.hpp>
#include "Plugin.hpp"


/**
 * This program demonstrates the use of the DynamicLibraryLoader to
 * load in libraries
 */
int main(int argc, char** argv) {
    //Load PluginA  
    rwlibs::dll::DynamicLibraryLoader<Plugin> factoryA("libPluginA", "factory0");
    Plugin* pluginA = factoryA.get();
	if (pluginA != NULL)
		std::cout<<"PluginA = "<<pluginA->name()<<std::endl;

	//Load PluginB  
	rwlibs::dll::DynamicLibraryLoader<Plugin> factoryB("libPluginB", "factory0");
    Plugin* pluginB = factoryB.get();
	if (pluginB != NULL)
		std::cout<<"PluginB = "<<pluginB->name()<<std::endl;
		
    return 0;
   
}
