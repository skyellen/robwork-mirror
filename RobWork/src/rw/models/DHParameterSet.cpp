#include "DHParameterSet.hpp"
#include <rw/models/Joint.hpp>
#include <rw/models/Accessor.hpp>
#include <boost/foreach.hpp>
using namespace rw::models;




std::vector<DHParameterSet> DHParameterSet::getDHParameters(SerialDevicePtr device) {
	std::vector<DHParameterSet> dhset;
	BOOST_FOREACH(Joint *joint, device->getJoints()){
        if( Accessor::dhSet().has( *joint ) ){
            dhset.push_back( Accessor::dhSet().get( *joint ) );
        }
	}
	return dhset;
}