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

#include "CompositeContactFilter.hpp"

#include <cstddef>

using namespace rw::graspplanning;

CompositeContactFilter::CompositeContactFilter(bool enableFullStats):
    _fullTestEnabled(enableFullStats)
{

}

bool CompositeContactFilter::isValid(const rw::sensor::Contact3D& con){
    _nrOfTests++;
    bool valid = true;

    if(!_fullTestEnabled){
        for(size_t i=0; i<_cfilters.size(); i++){
            if( !_cfilters[i]->isValid(con) ){
                _stats[i]++;
                valid = false;
            }
        }
    } else {
        for(size_t i=0; i<_cfilters.size(); i++){
            if( !_cfilters[i]->isValid(con) ){
                _stats[i]++;
                valid = false;
            }
        }
    }
    return valid;
}

void CompositeContactFilter::addFilter(ContactValidateFilter* filter){
    _cfilters.push_back( filter);
    _stats.push_back(0);
}

void CompositeContactFilter::setFilters(std::vector<ContactValidateFilter*> filters){
    _cfilters = filters;
    _stats.clear();
    _stats.resize(filters.size(), 0);
}
