/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#include "CompositeGraspFilter.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>
#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::graspplanning;

CompositeGraspFilter::CompositeGraspFilter(bool enableFullStats):
    _fullTestEnabled(enableFullStats)
{

}

bool CompositeGraspFilter::isValid(const Grasp3D& con){
    _nrOfTests++;
    bool valid = true;

    if(!_fullTestEnabled){
        for(size_t i=0; i<_gfilters.size(); i++){
            if( !_gfilters[i]->isValid(con) ){
                _stats[i]++;
                valid = false;
            }
        }
    } else {
        for(size_t i=0; i<_gfilters.size(); i++){
            if( !_gfilters[i]->isValid(con) ){
                _stats[i]++;
                valid = false;
            }
        }
    }
    return valid;
}


void CompositeGraspFilter::addFilter(GraspValidateFilter* filter){
    _gfilters.push_back( filter);
    _stats.push_back(0);
}

void CompositeGraspFilter::setFilters(std::vector<GraspValidateFilter*> filters){
    _gfilters = filters;
    _stats.clear();
    _stats.resize(filters.size(), 0);
}
