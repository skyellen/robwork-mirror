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

#include "Timer.hpp"
#include "TimerUtil.hpp"

#include <iostream>

using namespace rw::common;

Timer::Timer()
{
    reset();
}

Timer::~Timer()
{}

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
        return _totalTime * 0.001; // convert to se
    else
        return (_totalTime + TimerUtil::currentTimeMs() - _relativeTime)* 0.001;
}
