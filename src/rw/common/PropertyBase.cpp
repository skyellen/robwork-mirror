/*********************************************************************
 * RobWork Version 0.3
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


const PropertyType& PropertyBase::getType() const {
    return _propertyType;
}
