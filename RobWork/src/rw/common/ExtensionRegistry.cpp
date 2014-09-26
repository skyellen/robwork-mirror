#include "ExtensionRegistry.hpp"

#include "Plugin.hpp"

using namespace rw::common;

rw::common::Ptr<ExtensionRegistry> ExtensionRegistry::getInstance(){
    static rw::common::Ptr<ExtensionRegistry> _instance;
    if(_instance == NULL)
    	_instance = rw::common::ownedPtr(new ExtensionRegistry());
    return _instance;
}

ExtensionRegistry::ExtensionRegistry() {
    // load settings file

}

std::vector<Extension::Descriptor> ExtensionRegistry::getExtensionDescriptors(const std::string& ext_point_id) const {
	std::vector<Extension::Descriptor> result;

	std::map<std::string, std::vector< std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > > >::const_iterator it;
	it = _descMap.find(ext_point_id);
	if(it==_descMap.end())
		return std::vector<Extension::Descriptor>();

	typedef std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > Desc;
	BOOST_FOREACH(const Desc &desc, it->second){
		result.push_back( desc.first );
	}
	return result;
}

std::vector<rw::common::Ptr<Extension> > ExtensionRegistry::getExtensions(const std::string& ext_point_id) const {
	std::vector<rw::common::Ptr<Extension> > result;

	std::map<std::string, std::vector< std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > > >::const_iterator it;
	it = _descMap.find(ext_point_id);
	if(it==_descMap.end())
		return result;

	typedef std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > Desc;
	BOOST_FOREACH(const Desc &desc, it->second){
		result.push_back( desc.second->makeExtension( desc.first.id ) );
	}
	return result;
}


void ExtensionRegistry::registerExtensions(rw::common::Ptr<Plugin> plugin){
	BOOST_FOREACH(Extension::Descriptor desc, plugin->getExtensionDescriptors()){
		_descMap[desc.point].push_back( std::make_pair( desc , plugin ) );
	}
	_plugins.push_back( plugin );
}
