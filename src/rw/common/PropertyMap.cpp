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
#include <boost/foreach.hpp>

using namespace rw::common;
using namespace boost;

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
    BOOST_FOREACH(PropertyBasePtr base, other._properties) {
        this->insert(rw::common::Ptr<PropertyBase>(base->clone()));
    }
}

bool PropertyMap::add(PropertyBasePtr property) {
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

size_t PropertyMap::size() const
{
    return _properties.size();
}

bool PropertyMap::empty() const
{
    return _properties.empty();
}

bool PropertyMap::insert(PropertyBasePtr property)
{
    return _properties.insert(property).second;
}

PropertyBase* PropertyMap::findPropertyBase(const std::string& identifier)
{
    Property<int> key(identifier, "", 0);
    typedef MapType::iterator I;
    const I p = _properties.find(&key);
    if (p != _properties.end())
        return (*p).get();
    return NULL;
}

const PropertyBase* PropertyMap::findPropertyBase(const std::string& identifier) const
{
    return const_cast<PropertyMap*>(this)->findPropertyBase(identifier);
}

std::pair<PropertyMap::iterator, PropertyMap::iterator>
PropertyMap::getProperties() const
{
    return std::make_pair(_properties.begin(), _properties.end());
}
