#include "AnimalFactory.hpp"

#include "Cat.hpp"

using std::vector;
using namespace animals;
using rw::common::ownedPtr;
using rw::common::ExtensionPoint;
using rw::common::Extension;



AnimalFactory::AnimalFactory() :
	ExtensionPoint<Animal>("plugintest.animals.Animal", "extension point for new animal species")
{
}


Animal::Ptr AnimalFactory::getAnimal(const std::string& species) {
	/*
	 * Check if the requested Animal is available statically.
	 */
	if (species == "cat") {
		return ownedPtr(new Cat());
	}
	
	/*
	 * Check if the requested Animal is available from extensions.
	 */
	AnimalFactory animalFactory;
	vector<Extension::Ptr> extensions = animalFactory.getExtensions();
	
	BOOST_FOREACH (Extension::Ptr& extension, extensions) {
		
		if (extension->getProperties().get("species", extension->getName()) == species) {
			
			return extension->getObject().cast<Animal>();
		}
	}
	
	/*
	 * At this point, it is certain that the requested type of Animal is not available.
	 */
	return NULL;
}
