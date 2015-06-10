#include "DogPlugin.hpp"

#include "Dog.hpp"



using namespace std;
using namespace animals;
USE_ROBWORK_NAMESPACE
using namespace robwork;


RW_ADD_PLUGIN(DogPlugin)



DogPlugin::DogPlugin() :
	Plugin("DogPlugin", "DogPlugin", "1.0")
{
}


DogPlugin::~DogPlugin() {
}


vector<Extension::Descriptor> DogPlugin::getExtensionDescriptors() {
	
	vector<Extension::Descriptor> descriptors;
	
	/*
	 * We define the extension descriptor for this plugin.
	 * The extension name is 'Dog', and it extends the 'plugintest.animals.Animal' extension point.
	 * We add property 'species' with value of 'dog', so that it can be looked up
	 * in the extension registry.
	 * Plugin can contain many extensions, hence description vector.
	 */
	Extension::Descriptor descriptor("Dog", "plugintest.animals.Animal");
	descriptor.getProperties().set<string>("species", "dog");
	
	descriptors.push_back(descriptor);
	
	return descriptors;
}


Ptr<Extension> DogPlugin::makeExtension(const string& str) {
	
	if (str == "Dog") {
		
		/*
		 * Create the instance of extension when requested.
		 * We have to specify extension name, extension point, plugin
		 * instance, and contents of the extension.
		 * Plugin may contain many extensions, hence the 'if'.
		 */
		Extension::Ptr extension = ownedPtr(new Extension(
			"Dog",							// extension name
			"plugintest.animals.Animal",	// extension point
			this,							// instance of plugin
			ownedPtr(new Dog())				// contents of the extension (object)
		));
		extension->getProperties().set<string>("species", "dog");
		
		return extension;
	}
	
	return NULL;
}
