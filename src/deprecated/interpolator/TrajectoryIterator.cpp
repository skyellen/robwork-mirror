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

#include "TrajectoryIterator.hpp"

using namespace rw::interpolator;

TrajectoryIterator::TrajectoryIterator(const std::vector<Interpolator*>* interpolators):
    _interpolators(interpolators),_currIndex(0),_currOffset(0),_endReached(false),
    _beginReached(true),_currIIterator(NULL)
{
    if(!_interpolators->empty()){
        _currIIterator = (*_interpolators)[0]->getIterator();
    }
}

void TrajectoryIterator::operator -=(double val){
    (*this) += (-val);
}

void TrajectoryIterator::operator +=(double val){
    double currLength = (*_interpolators)[_currIndex]->getLength();
    double newOffset = _currOffset+val;
    if(newOffset>currLength){// shift to next interpolator
        if( _currIndex==_interpolators->size()-1 ){ // is end reached
            _currOffset = currLength;
            _currIIterator+=currLength;
            _endReached = true;
            return;
        }
        _currOffset = 0;
        _currIndex++;
        _endReached = false;
        // else go to next interpolator
        _currIIterator = (*_interpolators)[_currIndex]->getIterator();
        (*this) += (newOffset-currLength);
    } else if(newOffset<0){ // shift to last interpolator
        if( _currIndex==0 ){
            _currOffset = 0;
            _currIIterator+=newOffset;
            _beginReached = true;
            return;
        }
        _currIndex--;
        _currOffset = (*_interpolators)[_currIndex]->getLength();
        _beginReached = false;
        // else go to last interpolator
        _currIIterator =  (*_interpolators)[_currIndex]->getIterator();
        (*this) += (newOffset-currLength);
    } else {
        _currOffset = newOffset;
        _currIIterator+=val;
    }
}
