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

#include "IterativeMultiIK.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/shared_ptr.hpp>

using namespace rw::invkin;
using namespace rw::common;
using namespace boost;

IterativeMultiIK::IterativeMultiIK(size_t nrOfEndEff) :
    _nrOfEndEff(nrOfEndEff)
{
    _properties.add(
        "MaxIterations", "Max number of iterations", 20);

    std::vector<double> maxError(_nrOfEndEff);
    for(size_t i=0;i<_nrOfEndEff;i++)
        maxError[i] = 1e-6;

    _properties.add(
        "MaxErrorVector", "Max Error ",maxError);
}


void IterativeMultiIK::setMaxError(const std::vector<double>& maxError)
{
    if( maxError.size()!=_nrOfEndEff )
        RW_THROW("Size of maxError vector, must equal nr of end effectors"<< _nrOfEndEff);

    for(size_t i=0;i<_nrOfEndEff;i++)
        if( maxError[i]<0 )
            RW_THROW("MaxError must be positive");

    _properties.set("MaxErrorVector", maxError);
}

std::vector<double> IterativeMultiIK::getMaxError() const
{
    return _properties.get<std::vector<double> >("MaxErrorVector");
}

void IterativeMultiIK::setMaxIterations(int maxIterations)
{
    _properties.set("MaxIterations", maxIterations);
}

int IterativeMultiIK::getMaxIterations() const
{
    return _properties.get<int>("MaxIterations");
}

PropertyMap& IterativeMultiIK::getProperties()
{
    return _properties;
}

const PropertyMap& IterativeMultiIK::getProperties() const
{
    return _properties;
}
