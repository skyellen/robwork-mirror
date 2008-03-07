/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PropertyMap.hpp"
#include <boost/foreach.hpp>

using namespace rw::common;
using namespace boost;

PropertyMap::PropertyMap() {}

PropertyMap::~PropertyMap()
{
    // Delete all property base objects.
    BOOST_FOREACH(PropertyBase* base, _properties) {
        delete base;
    }
}

PropertyMap::PropertyMap(const PropertyMap& other)
{
    // Clone all property base objects.
    BOOST_FOREACH(PropertyBase* base, other._properties) {
        this->insert(base->clone());
    }
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

bool PropertyMap::insert(PropertyBase* property)
{
    return _properties.insert(property).second;
}

PropertyBase* PropertyMap::findPropertyBase(const std::string& identifier)
{
    Property<int> key(identifier, "", 0);
    typedef MapType::iterator I;
    const I p = _properties.find(&key);
    if (p != _properties.end())
        return *p;
    return NULL;
}

const PropertyBase* PropertyMap::findPropertyBase(
    const std::string& identifier) const
{
    return const_cast<PropertyMap*>(this)->findPropertyBase(identifier);
}

std::pair<PropertyMap::iterator, PropertyMap::iterator>
PropertyMap::getProperties() const
{
    return std::make_pair(_properties.begin(), _properties.end());
}

//----------------------------------------------------------------------

const PropertyBase* PropertyMap::find(const std::string& identifier) const
{
    return findPropertyBase(identifier);
}

PropertyBase* PropertyMap::find(const std::string& identifier)
{
    return findPropertyBase(identifier);
}

bool PropertyMap::removeProperty(const std::string& identifier)
{
    return erase(identifier);
}
