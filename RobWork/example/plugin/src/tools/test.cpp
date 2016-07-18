#include "path.hpp"

#include <iostream>
#include <rw/RobWork.hpp>
#include <rw/common/Plugin.hpp>

#include <animals.hpp>



using namespace std;
using namespace animals;
using rw::RobWork;
using namespace rw::common;



int main(int argc, char* argv[]) {
	/*
	 * Initialize RobWork and let it search for plugins.
	 */
	RobWork::getInstance()->initialize(std::vector<std::string>(1,DOG_PLUGIN_PATH));
	
	/*
	 * List all the plugins used.
	 */
	ExtensionRegistry::Ptr extReg = ExtensionRegistry::getInstance();
	vector<Ptr<Plugin> > plugins = extReg->getPlugins();
	
	cout << "List of plugins: " << endl;
	BOOST_FOREACH (Ptr<Plugin>& plugin, plugins) {
		cout << " + " << plugin->getName() << endl;
	}
	
	/*
	 * Construct new Cat, which is available statically.
	 */
	Animal::Ptr cat = AnimalFactory::getAnimal("cat");
	cat->makeSound();
	
	/*
	 * Construct new Dog. This will only work if the dog plugin is compiled and found.
	 * Look up RobWork .cfg.xml file to see where to put the dog plugin .so.
	 */
	Animal::Ptr dog = AnimalFactory::getAnimal("dog");
	if (dog.isNull())
		RW_THROW("Plugin was not found!");
	dog->makeSound();
	
	return 0;
}
