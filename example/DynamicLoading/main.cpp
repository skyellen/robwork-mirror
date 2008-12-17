#include <iostream>

#include <rwlibs/dll/DynamicLibraryLoader.hpp>
#include "Plugin.hpp"


#include <rw/kinematics/MovableFrame.hpp>
using namespace rw::kinematics;
/**
 * This program demonstrates the use of the DynamicLibraryLoader to
 * load in libraries
 */
int main(int argc, char** argv) {
    //Load PluginA
    std::cout << "Loading pluginA" << std::endl;
	rwlibs::dll::DynamicLibraryLoader<Plugin> factoryA("libPluginA", "factory0");
    Plugin* pluginA = factoryA.get();
	if (pluginA != NULL)
		std::cout<<"PluginA = "<<pluginA->name()<<std::endl;

	//Load PluginB
	std::cout << "Loading pluginB" << std::endl;
	rwlibs::dll::DynamicLibraryLoader<Plugin> factoryB("libPluginB", "factory0");
    Plugin* pluginB = factoryB.get();
	if (pluginB != NULL)
		std::cout<<"PluginB = "<<pluginB->name()<<std::endl;

	std::cout << "Get obj from pluginA" << std::endl;
	Frame *obj = pluginA->getObj();
	MovableFrame *vobj = dynamic_cast<MovableFrame*> (obj);
	if( vobj==NULL )
		std::cout << "MAIN: could not dynamic cast object from PluginA!!" << std::endl;

	std::cout << "Set obj in pluginB" << std::endl;
	pluginB->setObj(obj);

	std::cout << "Set obj in pluginA" << std::endl;

	pluginA->setObj( pluginB->getObj() );
    return 0;

}
