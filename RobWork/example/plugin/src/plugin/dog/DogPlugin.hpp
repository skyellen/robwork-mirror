#ifndef __DOG_PLUGIN_HPP
#define __DOG_PLUGIN_HPP

#include <rw/common/Plugin.hpp>



/**
 * Defines a plugin that adds Dog species to AnimalFactory.
 */
class DogPlugin: public rw::common::Plugin {
public:

	DogPlugin();
	
	~DogPlugin();
	
	//! Override Plugin::getExtensionDescriptors
	std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();
	
	//! Override Plugin::makeExtension
	rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);
};

#endif
