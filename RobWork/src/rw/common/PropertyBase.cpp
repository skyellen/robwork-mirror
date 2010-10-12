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


#include "PropertyBase.hpp"

using namespace rw::common;

PropertyBase::PropertyBase(const std::string& identifier,
                           const std::string& description):
    _identifier(identifier),
    _description(description)
{
}



PropertyBase::PropertyBase(const std::string& identifier,
                           const std::string& description,
                           const PropertyType& propertyType):
    _identifier(identifier),
    _description(description),
    _propertyType(propertyType)
{
}

PropertyBase::~PropertyBase() {}

const std::string& PropertyBase::getIdentifier() const {
    return _identifier;
}

const std::string& PropertyBase::getDescription() const {
    return _description;
}


void PropertyBase::notifyListeners() {
    typedef std::vector<PropertyChangedListener>::iterator I;
    for (I it = _listeners.begin(); it != _listeners.end(); ++it) {
        (*it)(this);
    }
}

void PropertyBase::addChangedListener(PropertyChangedListener callback) {
    _listeners.push_back(callback);
}

void PropertyBase::removeChangedListener(PropertyChangedListener callback){
    std::vector<PropertyChangedListener>::iterator iter = _listeners.begin();



    for(;iter!=_listeners.end();++iter ){
        //std::cout << "Typeid: " << (*iter).target_type().name()
        //          << " == " << callback.target_type().name()
        //          << std::endl;


        //if( (*iter).target_type() == callback.target_type() ){
        //        _listeners.erase(iter);
        //        return;
        //}

        //if(*iter == callback ){
        //    _listeners.erase(iter);
        //    return;
        //}
    }

}

const PropertyType& PropertyBase::getType() const {
    return _propertyType;
}
