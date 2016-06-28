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


#include "IterativeMultiIK.hpp"

#include <rw/common/macros.hpp>

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
