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

#include "IterativeMultiIK.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/shared_ptr.hpp>

using namespace rw::invkin;
using namespace rw::common;
using namespace boost;

IterativeMultiIK::IterativeMultiIK(size_t nrOfEndEff):_nrOfEndEff(nrOfEndEff) {
    _properties.addProperty(shared_ptr<Property<unsigned int> >(new Property<unsigned int>("MaxIterations",
                                               "Max number of iterations",
                                               20)));
    
    std::vector<double> maxError(_nrOfEndEff);
    for(size_t i=0;i<_nrOfEndEff;i++)
        maxError[i] = 1e-6;
    
    _properties.addProperty(shared_ptr<Property<std::vector<double> > > (
            new Property<std::vector<double> >("MaxErrorVector", "Max Error ",maxError)));
    
}


void IterativeMultiIK::setMaxError(const std::vector<double>& maxError) {
    if( maxError.size()!=_nrOfEndEff )
        RW_THROW("Size of maxError vector, must equal nr of end effectors"<< _nrOfEndEff);
    
    for(size_t i=0;i<_nrOfEndEff;i++)
        if( maxError[i]<0 )
            RW_THROW("MaxError must be positive");

    _properties.getProperty<std::vector<double> >("MaxErrorVector")->setValue(maxError);
}

std::vector<double> IterativeMultiIK::getMaxError() const {
    const Property<std::vector<double> >* property = 
        _properties.getProperty<std::vector<double> >("MaxErrorVector");
    return property->getValue();
}

void IterativeMultiIK::setMaxIterations(unsigned int maxIterations) {
    _properties.getProperty<unsigned int>("MaxIterations")->setValue(maxIterations);
}

unsigned int IterativeMultiIK::getMaxIterations() const {
    return _properties.getProperty<unsigned int>("MaxIterations")->getValue();
}


PropertyMap& IterativeMultiIK::getProperties() {
    return _properties;
}

const PropertyMap& IterativeMultiIK::getProperties() const {
    return _properties;
}
