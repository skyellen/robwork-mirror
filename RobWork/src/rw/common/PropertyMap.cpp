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


#include "PropertyMap.hpp"

using namespace rw::common;

PropertyMap::PropertyMap() {}

PropertyMap::~PropertyMap()
{
    _properties.clear();
    // Delete all property base objects.
    /*BOOST_FOREACH(PropertyBase* base, _properties) {
        delete base;
    }*/
}

PropertyMap::PropertyMap(const PropertyMap& other)
{
    // Clone all property base objects.
	for (MapType::iterator it = other._properties.begin(); it != other._properties.end(); it++) {
		const PropertyBase::Ptr base = *it;
        this->insert(ownedPtr(base->clone()));
    }
}

bool PropertyMap::add(PropertyBase::Ptr property) {
    return insert(property);
}

PropertyMap& PropertyMap::operator=(const PropertyMap& other)
{
    // Assignment operator by the swap-idiom.
    if (this != &other) {
        PropertyMap copy = other;
        swap(copy);
    }
    return *this;
}

void PropertyMap::swap(PropertyMap& other)
{
    _properties.swap(other._properties);

    // all properties need to update their actionhandler
  /*  Range r = this->getProperties();
    for(;r.first!=r.second;++r.first){
        (*r.first)->changedEvent().remove( &other );
        (*r.first)->changedEvent().add( boost::bind(&PropertyMap::propertyChangedListener,this,_1), this );
    }

    r = other.getProperties();
    for(;r.first!=r.second;++r.first){
        (*r.first)->changedEvent().remove( this );
        (*r.first)->changedEvent().add( boost::bind(&PropertyMap::propertyChangedListener,&other,_1), &other );
    }*/
}

bool PropertyMap::has(const std::string& identifier) const
{
    return findPropertyBase(identifier) != NULL;
}

bool PropertyMap::erase(const std::string& identifier)
{
    Property<int> key(identifier, "", 0);

    typedef MapType::iterator I;
    const I p = _properties.find(&key);
    if (p != _properties.end()) {
        _properties.erase(p);
        return true;
    } else {
        return false;
    }
}


void PropertyMap::clear()
{
	_properties.clear();
}

size_t PropertyMap::size() const
{
    return _properties.size();
}

bool PropertyMap::empty() const
{
    return _properties.empty();
}

bool PropertyMap::insert(PropertyBase::Ptr property)
{
    if( _properties.insert(property).second ){
        // add to changed listener
        //property->addChangedListener( boost::bind(&PropertyMap::propertyChangedListener,this,_1) );
        return true;
    }
    return false;
}

PropertyBase::Ptr PropertyMap::findPropertyBase(const std::string& identifier)
{
    Property<int> key(identifier, "", 0);
    typedef MapType::iterator I;
    const I p = _properties.find(&key);
    if (p != _properties.end())
        return *p;
    return NULL;
}

const PropertyBase::Ptr PropertyMap::findPropertyBase(const std::string& identifier) const
{
    return const_cast<PropertyMap*>(this)->findPropertyBase(identifier);
}

std::pair<PropertyMap::iterator, PropertyMap::iterator>
PropertyMap::getProperties() const
{
    return std::make_pair(_properties.begin(), _properties.end());
}

void PropertyMap::notifyListeners(PropertyBase* base) {
    typedef std::vector<PropertyChangedListener>::iterator I;
    for (I it = _listeners.begin(); it != _listeners.end(); ++it) {
        (*it)(this, base);
    }
}

void PropertyMap::addChangedListener(PropertyChangedListener callback) {
    _listeners.push_back(callback);
}

void PropertyMap::propertyChangedListener(PropertyBase* base){
    std::string id = base->getIdentifier();
    //std::cout << "PropertyMap: Property Changed Listerner: " << id << std::endl;
    // notify all listeners
    notifyListeners(base);
}


