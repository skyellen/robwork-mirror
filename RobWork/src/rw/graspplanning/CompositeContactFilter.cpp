/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#include "CompositeContactFilter.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
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
