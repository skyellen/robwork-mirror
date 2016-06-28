/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "DHParameterSet.hpp"
#include <rw/models/Joint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <boost/foreach.hpp>
using namespace rw::models;

std::vector<DHParameterSet> DHParameterSet::getDHParameters(SerialDevice::Ptr device) {
	std::vector<DHParameterSet> dhset;
	BOOST_FOREACH(Joint *joint, device->getJoints()){
        if( get( joint )!=NULL ){
            dhset.push_back( *get( joint ) );
        }
	}
	return dhset;
}

const DHParameterSet* DHParameterSet::get(const rw::models::Joint* joint) {
    return get(joint->getPropertyMap());
}

const DHParameterSet* DHParameterSet::get(const rw::common::PropertyMap& pmap) {
    return pmap.getPtr<DHParameterSet>("DHSet");
}

void DHParameterSet::set(const DHParameterSet& dhset, rw::kinematics::Frame* joint){
    set(dhset, joint->getPropertyMap());
}

void DHParameterSet::set(const DHParameterSet& dhset, rw::common::PropertyMap& pmap){
    pmap.addForce<DHParameterSet>("DHSet","Denavit-Hartenberg parameters",dhset);
}
