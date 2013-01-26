#include "Model3DLoader.hpp"

using namespace rw::loaders;

rw::common::Ptr<Model3DLoader> Model3DLoader::Factory::getModel3DLoader(const std::string& format){
	using namespace rw::common;
	Model3DLoader::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(!ext->getProperties().has(format))
			continue;
		// else try casting to ImageLoader
		Model3DLoader::Ptr loader = ext->getObject().cast<Model3DLoader>( );

		return loader;
	}
	RW_THROW("No loader using that format exists...");
	return NULL;
}

bool Model3DLoader::Factory::hasModel3DLoader(const std::string& format){
	using namespace rw::common;
	Model3DLoader::Factory ep;
	std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		if(!ext.getProperties().has(format))
			continue;
		return true;
	}
	return false;
}
