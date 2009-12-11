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


#include "Timer.hpp"
#include "TimerUtil.hpp"

#include <iostream>

using namespace rw::common;

Timer::Timer()
{
    reset();
}

Timer::~Timer()
{

}

void Timer::reset()
{
    _relativeTime = TimerUtil::currentTimeMs();
    _totalTime = 0;
    _isPaused = false;
}

void Timer::pause()
{
    if (!_isPaused) {
        const long now = TimerUtil::currentTimeMs();
        _totalTime += now - _relativeTime;
        _relativeTime = now;
        _isPaused = true;
    }
}

void Timer::resume()
{
    if (_isPaused) {
        _relativeTime = TimerUtil::currentTimeMs();
        _isPaused = false;
    }
}



double Timer::getTime() const
{
    if (_isPaused)
        return _totalTime * 0.001; // convert to sec
    else
        return (_totalTime + TimerUtil::currentTimeMs() - _relativeTime)* 0.001;
}
