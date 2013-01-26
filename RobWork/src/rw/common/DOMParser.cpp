#include "DOMParser.hpp"

#include "BoostXMLParser.hpp"

using namespace rw::common;

rw::common::Ptr<DOMParser> DOMParser::make(){
	return rw::common::ownedPtr( new BoostXMLParser() );
}
