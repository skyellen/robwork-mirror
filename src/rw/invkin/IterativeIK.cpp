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

#include "IterativeIK.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/shared_ptr.hpp>

using namespace rw::invkin;
using namespace rw::common;
using namespace boost;

IterativeIK::IterativeIK() {
    _properties.addProperty(shared_ptr<Property<unsigned int> >(new Property<unsigned int>("MaxIterations",
                                               "Max number of iterations",
                                               20)));

    _properties.addProperty(shared_ptr<Property<double> > (new Property<double>("MaxError", "Max Error ", 1e-6)));

}


void IterativeIK::setMaxError(double maxError) {
    if (maxError < 0)
    RW_THROW("MaxError must be positive");

    _properties.getProperty<double>("MaxError")->setValue(maxError);
}

double IterativeIK::getMaxError() const {
    const Property<double>* property = _properties.getProperty<double>("MaxError");
    return property->getValue();
}

void IterativeIK::setMaxIterations(unsigned int maxIterations) {
    _properties.getProperty<unsigned int>("MaxIterations")->setValue(maxIterations);
}

unsigned int IterativeIK::getMaxIterations() const {
    return _properties.getProperty<unsigned int>("MaxIterations")->getValue();
}


PropertyMap& IterativeIK::getProperties() {
    return _properties;
}

const PropertyMap& IterativeIK::getProperties() const {
    return _properties;
}
