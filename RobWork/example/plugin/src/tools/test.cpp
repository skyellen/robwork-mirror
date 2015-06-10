#include <iostream>
#include <rw/RobWork.hpp>
#include <rw/rw.hpp>

#include <animals.hpp>



using namespace std;
using namespace animals;
USE_ROBWORK_NAMESPACE
using namespace robwork;



int main(int argc, char* argv[]) {
	/*
	 * Initialize RobWork and let it search for plugins.
	 */
	RobWork::getInstance()->initialize();
	
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
	dog->makeSound();
	
	return 0;
}
