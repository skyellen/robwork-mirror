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
