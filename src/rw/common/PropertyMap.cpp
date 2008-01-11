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

using namespace rw::common;
using namespace boost;

PropertyMap::PropertyMap() {}

PropertyMap::~PropertyMap() {}

bool PropertyMap::has(const std::string& identifier) const
{
    return find(identifier) != NULL;
}

const PropertyBase* PropertyMap::find(const std::string& identifier) const
{
    typedef MapType::const_iterator I;
    const I p = _properties.find(identifier);
    if (p != _properties.end())
        return (*p).second.get();
    return NULL;
}

PropertyBase* PropertyMap::find(const std::string& identifier)
{
    typedef MapType::iterator I;
    const I p = _properties.find(identifier);
    if (p != _properties.end())
        return (*p).second.get();
    return NULL;
}

bool PropertyMap::addProperty(shared_ptr<PropertyBase> property)
{
    return _properties.insert(
        std::make_pair(
            property->getIdentifier(),
            property)).second;
}

bool PropertyMap::removeProperty(const std::string& identifier)
{
    typedef MapType::iterator I;
    const I it = _properties.find(identifier);
    if (it != _properties.end()) {
        _properties.erase(it);
        return true;
    }
    return false;
}

size_t PropertyMap::size() const
{
    return _properties.size();
}

std::vector<shared_ptr<PropertyBase> > PropertyMap::properties() const
{
    std::vector<shared_ptr<PropertyBase> > result;
    typedef MapType::const_iterator I;
    for (I it = _properties.begin(); it != _properties.end(); ++it) {
        result.push_back(it->second);
    }
    return result;
}
