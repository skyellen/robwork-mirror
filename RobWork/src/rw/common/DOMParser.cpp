#include "DOMParser.hpp"

#include "BoostXMLParser.hpp"
#include "StringUtil.hpp"

using namespace rw::common;

rw::common::Ptr<DOMParser> DOMParser::make(){
	return rw::common::ownedPtr( new BoostXMLParser() );
}


rw::common::Ptr<DOMParser> DOMParser::Factory::getDOMParser(const std::string& format){
	using namespace rw::common;
	DOMParser::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(!ext->getProperties().has(format))
			continue;
		// else try casting to DOMParser
		DOMParser::Ptr loader = ext->getObject().cast<DOMParser>();
		return loader;
	}
	if(StringUtil::toUpper(format)=="XML")
		return rw::common::ownedPtr( new BoostXMLParser() );
	RW_THROW("No loader using that format exists...");
	return NULL;
}

bool DOMParser::Factory::hasDOMParser(const std::string& format){
	using namespace rw::common;

	if(StringUtil::toUpper(format)=="XML")
		return true;

	DOMParser::Factory ep;
	std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		if(!ext.getProperties().has(format))
			continue;
		return true;
	}
	return false;
}
