#include "ExtensionRegistry.hpp"

#include "Plugin.hpp"
#include <rw/RobWork.hpp>

using namespace rw::common;

rw::common::Ptr<ExtensionRegistry> ExtensionRegistry::getInstance(){
	return rw::RobWork::getInstance()->getExtensionRegistry();
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



std::vector<rw::common::Ptr<Plugin> > ExtensionRegistry::getPlugins() const {
	std::vector<rw::common::Ptr<Plugin> > result;
	
	BOOST_FOREACH (const rw::common::Ptr<Plugin> plugin, _plugins) {
		result.push_back(plugin);
	}
	
	return result;
}


void ExtensionRegistry::registerExtensions(rw::common::Ptr<Plugin> plugin){
	// make sure plugins with same id does not add duplicates of extension points
	BOOST_FOREACH(Extension::Descriptor desc, plugin->getExtensionDescriptors()){
		if(_descMap.find(desc.point)!=_descMap.end() ){
			// check if plugin/extension allready exists
			std::vector< std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > > &res = _descMap[desc.point];
			for(size_t i=0;i<res.size();i++){
				if(res[i].first.id == desc.id && res[i].second->getId() == plugin->getId()){
					continue;
				}
			}
		}
		_descMap[desc.point].push_back( std::make_pair( desc , plugin ) );
	}
	_plugins.push_back( plugin );
}
